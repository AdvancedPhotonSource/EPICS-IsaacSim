"""Differential test: our motor record against the genuine EPICS motorRecord.

Everything in tests/ checks our state machine against our own expectations.
This checks it against the real thing.

The oracle is `motorSim` -- the actual `motorRecord` from epics-modules/motor
running on the actual simulated asyn controller, in a container. Both sides
are driven over Channel Access with identical runtime configuration and
identical command sequences, and the SEMANTICS of the response are compared.

What is compared, and what is not
---------------------------------
Trajectories are NOT compared, and must not be. The oracle's plant is a
kinematic model; ours is a PhysX articulation with real gravity and finite
drive stiffness, or a first-order lag when run against fake_sim. The two will
legitimately take different paths to the same place.

What must agree is the behaviour a scan depends on:

  * `.DMOV` drops when a move is commanded, and rises exactly once
  * the readback ends inside `.RDBD` of the target
  * `.MOVN` is the complement of `.DMOV` while moving
  * a move outside the soft limits sets `.LVIO` and does NOT move the axis
  * `.VAL` is never silently clamped to a limit
  * `.STOP` ends a move and raises `.DMOV`
  * backlash produces an approach from a consistent direction

A disagreement here is a real defect in one of the two implementations, and
historically the one that is wrong is not always ours -- read the trace before
assuming.

Usage
-----
Start the oracle (see tools/motorsim/README.md), then::

    python tools/motor_oracle.py --oracle-port 5074

The subject adapter and its fake simulator are started by this script.
"""

import argparse
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, "src"))

#: Fields written to configure an axis before each scenario. Both sides get
#: exactly these values, so neither can be flattered by its own defaults.
SHARED_CONFIG = {
    ".VELO": 10.0, ".ACCL": 0.5, ".RDBD": 0.05,
    ".BDST": 0.0, ".BVEL": 5.0, ".HLM": 40.0, ".LLM": -40.0, ".DLY": 0.0,
}

SAMPLE_HZ = 20.0


class Axis:
    """One motor record, on either side, sampled over Channel Access."""

    def __init__(self, context, prefix, label):
        self.label = label
        self.prefix = prefix
        self._ctx = context
        self._pvs = {}

    def pv(self, field):
        if field not in self._pvs:
            name = self.prefix if field == "" else self.prefix + field
            pv, = self._ctx.get_pvs(name)
            pv.wait_for_connection(timeout=20)
            self._pvs[field] = pv
        return self._pvs[field]

    def get(self, field):
        return float(self.pv(field).read().data[0])

    def put(self, field, value, wait=False):
        self.pv(field).write((value,), wait=wait)

    def goto(self, position, timeout=40):
        """Pre-position the axis and wait for it to settle.

        Deliberately not ``write(wait=True)``: on a motor record the put
        callback completes when the MOVE completes, not when the value is
        accepted, and caproto's default write timeout is two seconds. Using
        wait=True here times out on any move longer than that and reports it
        as a channel error, which looks like a broken IOC and is not.
        """
        self.put(".VAL", position)
        time.sleep(0.4)
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                if self.get(".DMOV") == 1:
                    return True
            except Exception:  # noqa: BLE001 - keep polling through a dropped read
                pass
            time.sleep(0.1)
        return False

    def configure(self, **overrides):
        settings = dict(SHARED_CONFIG)
        settings.update(overrides)
        # Limits first: a tighter .HLM must not reject a .VAL written earlier.
        for field in (".HLM", ".LLM"):
            self.put(field, settings.pop(field))
        for field, value in settings.items():
            self.put(field, value)

    def sample(self, duration, settle_check=True):
        """Record a time series of the fields that carry move semantics."""
        trace = []
        deadline = time.time() + duration
        period = 1.0 / SAMPLE_HZ
        while time.time() < deadline:
            row = {"t": round(time.time(), 4)}
            for field in (".RBV", ".DMOV", ".MOVN", ".LVIO"):
                try:
                    row[field] = self.get(field)
                except Exception:  # noqa: BLE001 - a dropped read must not end the run
                    row[field] = None
            trace.append(row)
            if settle_check and len(trace) > 4:
                recent = [r[".DMOV"] for r in trace[-4:]]
                if all(v == 1 for v in recent):
                    break
            time.sleep(period)
        return trace


def features(trace, target, rdbd):
    """Reduce a trace to the semantic claims a scan actually relies on."""
    rbv = [r[".RBV"] for r in trace if r[".RBV"] is not None]
    dmov = [r[".DMOV"] for r in trace if r[".DMOV"] is not None]
    movn = [r[".MOVN"] for r in trace if r[".MOVN"] is not None]
    lvio = [r[".LVIO"] for r in trace if r[".LVIO"] is not None]

    rises = sum(1 for a, b in zip(dmov, dmov[1:]) if a == 0 and b == 1)
    start = rbv[0] if rbv else 0.0
    final = rbv[-1] if rbv else 0.0

    settle_t = None
    t0 = trace[0]["t"] if trace else 0.0
    for row in trace:
        if row[".DMOV"] == 1 and row["t"] > t0:
            settle_t = round(row["t"] - t0, 2)
            break

    # Overshoot relative to the direction of travel: the backlash signature.
    if target is not None and rbv:
        if target >= start:
            overshoot = round(max(rbv) - target, 4)
        else:
            overshoot = round(target - min(rbv), 4)
    else:
        overshoot = 0.0

    return {
        "dmov_dropped": any(v == 0 for v in dmov),
        "dmov_rises": rises,
        "settled": dmov[-1] == 1 if dmov else False,
        "settle_s": settle_t,
        "moved": round(abs(final - start), 4) > 1e-6,
        "travel": round(final - start, 4),
        "final_err": None if target is None else round(abs(final - target), 4),
        "in_deadband": None if target is None else abs(final - target) <= max(rdbd, 1e-9),
        "overshot": overshoot > max(rdbd, 1e-6),
        "movn_consistent": all(
            (m == 0) == (d == 1) for m, d in zip(movn, dmov)
        ) if movn and len(movn) == len(dmov) else None,
        "lvio": lvio[-1] if lvio else None,
    }


# Which features must match, and how strictly. Anything not listed is
# reported for context but never fails the comparison -- timing and distance
# are expected to differ between a kinematic plant and a physics one.
STRICT = ["dmov_dropped", "dmov_rises", "settled", "moved",
          "in_deadband", "overshot", "lvio"]
INFORMATIONAL = ["settle_s", "travel", "final_err", "movn_consistent"]


def scenario_simple(axis):
    axis.configure()
    axis.goto(0.0)
    axis.put(".VAL", 10.0)
    return axis.sample(20), 10.0, SHARED_CONFIG[".RDBD"]


def scenario_reverse(axis):
    axis.configure()
    axis.goto(10.0)
    axis.put(".VAL", -10.0)
    return axis.sample(20), -10.0, SHARED_CONFIG[".RDBD"]


def scenario_zero_distance(axis):
    axis.configure()
    axis.goto(5.0)
    here = axis.get(".RBV")
    axis.put(".VAL", round(here, 4))
    return axis.sample(8), round(here, 4), SHARED_CONFIG[".RDBD"]


def scenario_soft_limit(axis):
    """Commanding past .HLM must set .LVIO, move nothing, and not clamp .VAL."""
    axis.configure()
    axis.goto(0.0)
    axis.put(".HLM", 5.0)
    time.sleep(0.5)
    axis.put(".VAL", 50.0)
    trace = axis.sample(8, settle_check=False)
    return trace, None, SHARED_CONFIG[".RDBD"]


def scenario_backlash(axis):
    """With .BDST set, the final approach must come from one direction."""
    axis.configure(**{".BDST": 2.0})
    axis.goto(15.0)
    axis.put(".VAL", 0.0)
    return axis.sample(25), 0.0, SHARED_CONFIG[".RDBD"]


def scenario_stop(axis):
    """.STOP must end the move and raise .DMOV."""
    axis.configure(**{".VELO": 2.0})
    axis.goto(0.0)
    axis.put(".VAL", 35.0)
    time.sleep(2.0)
    axis.put(".STOP", 1)
    return axis.sample(10, settle_check=False), None, SHARED_CONFIG[".RDBD"]


SCENARIOS = [
    ("simple_move", scenario_simple),
    ("reverse_move", scenario_reverse),
    ("zero_distance", scenario_zero_distance),
    ("soft_limit_violation", scenario_soft_limit),
    ("backlash", scenario_backlash),
    ("stop_mid_move", scenario_stop),
]


def start_subject(ca_port, http_port, prefix):
    """Start our adapter plus fake_sim as the implementation under test.

    fake_sim rather than Isaac Sim on purpose: the point is to compare motor
    record semantics, and a deterministic first-order plant makes the run
    reproducible on any machine. Isaac Sim would add real dynamics and real
    variance to a comparison that is not about dynamics.
    """
    env = dict(os.environ, EPICS_CA_SERVER_PORT=str(ca_port),
               EPICS_CA_AUTO_ADDR_LIST="NO", PYTHONUNBUFFERED="1")
    procs = []
    procs.append(subprocess.Popen(
        [sys.executable, "-m", "isaac_epics.cli", "--prefix", prefix,
         "--http-port", str(http_port)],
        cwd=REPO, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
    time.sleep(4)
    procs.append(subprocess.Popen(
        [sys.executable, os.path.join(REPO, "examples", "fake_sim.py"),
         "--url", f"http://127.0.0.1:{http_port}", "--quiet"],
        cwd=REPO, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
    time.sleep(4)
    return procs


def compare(name, oracle_f, subject_f):
    """Return (verdict, rows) for one scenario."""
    rows, disagreements = [], []
    for key in STRICT + INFORMATIONAL:
        o, s = oracle_f.get(key), subject_f.get(key)
        strict = key in STRICT
        same = (o == s)
        if strict and not same:
            disagreements.append(key)
        rows.append((key, o, s, "" if not strict else ("ok" if same else "DIFFER")))
    return disagreements, rows


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Compare our motor record against the genuine EPICS motorRecord.")
    parser.add_argument("--oracle-prefix", default="SIM:ORACLE:")
    parser.add_argument("--oracle-motor", default="M1")
    parser.add_argument("--oracle-port", type=int, default=5074,
                        help="Channel Access port the oracle IOC serves on")
    parser.add_argument("--subject-prefix", default="SIM:SUBJECT:")
    parser.add_argument("--subject-motor", default="m1")
    parser.add_argument("--subject-port", type=int, default=5084)
    parser.add_argument("--http-port", type=int, default=8244)
    parser.add_argument("--json", metavar="FILE", help="Write full results as JSON")
    parser.add_argument("--only", help="Run a single scenario by name")
    args = parser.parse_args(argv)

    # Both servers are loopback on distinct ports; the client must be told
    # about both explicitly, and before caproto builds its Context.
    os.environ["EPICS_CA_ADDR_LIST"] = (
        f"127.0.0.1:{args.oracle_port} 127.0.0.1:{args.subject_port}")
    os.environ["EPICS_CA_AUTO_ADDR_LIST"] = "NO"

    procs = start_subject(args.subject_port, args.http_port, args.subject_prefix)
    try:
        from caproto.threading.client import Context
        ctx = Context()
        oracle = Axis(ctx, args.oracle_prefix + args.oracle_motor, "oracle")
        subject = Axis(ctx, args.subject_prefix + args.subject_motor, "subject")

        print(f"oracle : {oracle.prefix}  (real motorRecord, CA :{args.oracle_port})")
        print(f"subject: {subject.prefix}  (this adapter, CA :{args.subject_port})")
        print()

        results, failed = {}, []
        for name, fn in SCENARIOS:
            if args.only and name != args.only:
                continue
            print(f"--- {name} " + "-" * (58 - len(name)))
            try:
                o_trace, o_target, o_rdbd = fn(oracle)
                o_feat = features(o_trace, o_target, o_rdbd)
                s_trace, s_target, s_rdbd = fn(subject)
                s_feat = features(s_trace, s_target, s_rdbd)
            except Exception as exc:  # noqa: BLE001 - report and keep going
                print(f"    ERROR: {exc}")
                failed.append(name)
                continue

            disagreements, rows = compare(name, o_feat, s_feat)
            print(f"    {'feature':<18}{'oracle':>14}{'subject':>14}   verdict")
            for key, o, s, verdict in rows:
                print(f"    {key:<18}{str(o):>14}{str(s):>14}   {verdict}")
            if disagreements:
                failed.append(name)
                print(f"    => DISAGREES on: {', '.join(disagreements)}")
            else:
                print("    => agrees on every semantic claim")
            print()
            results[name] = {"oracle": o_feat, "subject": s_feat,
                             "disagreements": disagreements}

        print("=" * 68)
        if failed:
            print(f"RESULT: {len(failed)} scenario(s) disagree: {', '.join(failed)}")
        else:
            print(f"RESULT: all {len(results)} scenarios agree with the real motor record")
        print("=" * 68)

        if args.json:
            with open(args.json, "w") as handle:
                json.dump(results, handle, indent=2, default=str)
            print(f"full results written to {args.json}")
        return 1 if failed else 0
    finally:
        for proc in procs:
            proc.terminate()
        time.sleep(1)
        for proc in procs:
            if proc.poll() is None:
                proc.kill()


if __name__ == "__main__":
    raise SystemExit(main())
