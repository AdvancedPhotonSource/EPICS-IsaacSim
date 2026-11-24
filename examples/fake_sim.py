"""A stand-in for Isaac Sim, for testing the adapter with no GPU and no simulator.

This runs the *same* ``epics_tick.EpicsTickClient`` that the real Isaac Sim
script uses, over the same HTTP protocol, against the real adapter.  What it
replaces is only the physics: joints follow their commanded targets through a
first-order lag instead of through a PhysX articulation.

That makes it useful for exactly two things:

* developing and testing the adapter, including on a laptop with no EPICS and
  no NVIDIA hardware;
* reproducing control-path behaviour (deadbands, ``.DMOV`` timing, staleness,
  pause semantics) without booking a GPU.

It is **not** a digital twin and proves nothing about geometry, collisions or
dynamics.  A green run here means the control path works; it does not mean the
model is right.  Section 5 of the design document is explicit that B raises the
fidelity of the control path, not of the model.

Usage::

    python examples/fake_sim.py --url http://127.0.0.1:8227
    python examples/fake_sim.py --pause-after 5     # test pause semantics
    python examples/fake_sim.py --die-after 10      # test staleness alarms
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), "..", "src", "isaac_epics", "isaacsim_client"))

from epics_tick import EpicsTickClient  # noqa: E402

#: Joint set matching the 4-DOF prototype stage: three prismatic joints
#: reported in metres and one revolute joint reported in radians.
DEFAULT_JOINTS = {
    "x_axis": 0.0,
    "y_axis": 0.0,
    "z_axis": 0.0,
    "theta_axis": 0.0,
}

#: Time constant of the joint servo, seconds.  A real articulation does not
#: reach its commanded target instantly; giving the fake one a lag is what
#: makes the adapter's deadband and retry logic actually get exercised.
TAU = 0.030


def main(argv=None):
    parser = argparse.ArgumentParser(description="Fake Isaac Sim client for the EPICS adapter")
    parser.add_argument("--url", default="http://127.0.0.1:8227")
    parser.add_argument("--rate", type=float, default=60.0, help="Simulated frame rate (Hz)")
    parser.add_argument("--tick-rate", type=float, default=30.0, help="Adapter exchange rate (Hz)")
    parser.add_argument("--tau", type=float, default=TAU, help="Joint lag time constant (s)")
    parser.add_argument("--noise", type=float, default=0.0,
                        help="Readback noise amplitude in sim units (metres/radians)")
    parser.add_argument("--pause-after", type=float, default=None,
                        help="Report playing=False after N seconds (tests pause semantics)")
    parser.add_argument("--resume-after", type=float, default=None,
                        help="Resume after N seconds")
    parser.add_argument("--die-after", type=float, default=None,
                        help="Stop posting frames after N seconds (tests staleness alarms)")
    parser.add_argument("--duration", type=float, default=None, help="Exit after N seconds")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    joints = dict(DEFAULT_JOINTS)
    frame_period = 1.0 / args.rate

    client = EpicsTickClient(args.url, rate_hz=args.tick_rate).start()
    if not args.quiet:
        print(f"fake sim: {len(joints)} joints -> {args.url} at {args.rate:g} Hz")
        print("          joints:", ", ".join(sorted(joints)))
        print("          this is NOT a twin - it tests the control path only")

    t_sim = 0.0
    started = time.time()
    last_report = 0.0
    rng_phase = 0.0

    try:
        while True:
            wall = time.time() - started
            if args.duration is not None and wall >= args.duration:
                break

            playing = True
            if args.pause_after is not None and wall >= args.pause_after:
                playing = False
                if args.resume_after is not None and wall >= args.resume_after:
                    playing = True

            dead = args.die_after is not None and wall >= args.die_after
            if dead:
                if not args.quiet and last_report >= 0:
                    print(f"[{wall:6.2f}s] simulating a dead link - no more frames")
                    last_report = -1.0
                time.sleep(frame_period)
                continue

            if playing:
                t_sim += frame_period
                targets = client.exchange(joints, t_sim, True)
                # First-order lag toward the commanded target.
                alpha = 1.0 - math.exp(-frame_period / max(args.tau, 1e-6))
                for joint, target in targets.items():
                    if joint in joints:
                        joints[joint] += (target - joints[joint]) * alpha
                if args.noise:
                    rng_phase += 0.37
                    for joint in joints:
                        joints[joint] += args.noise * math.sin(rng_phase * 7.3 + hash(joint) % 7)
            else:
                # Paused: the clock stops and the joints hold. The adapter must
                # refuse to complete a move in this state.
                client.exchange(joints, t_sim, False)

            if not args.quiet and wall - last_report >= 2.0 and last_report >= 0:
                stats = client.stats()
                state = "PLAYING" if playing else "PAUSED"
                pos = " ".join(f"{k}={v:+.5f}" for k, v in sorted(joints.items()))
                print(f"[{wall:6.2f}s] {state} t_sim={t_sim:7.2f} {pos} "
                      f"rtt={stats.round_trip*1e3:.1f}ms err={stats.errors}")
                last_report = wall

            time.sleep(frame_period)
    except KeyboardInterrupt:
        pass
    finally:
        client.stop()
        if not args.quiet:
            print("fake sim: stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
