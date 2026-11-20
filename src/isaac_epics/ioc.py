"""Channel Access layer: binds :mod:`isaac_epics.motor` to caproto.

``caproto.server.records.MotorFields`` gives us all 153 motor record fields
for free -- the correct names, types, enums and metadata links, generated from
the dbd.  What it does not give us is behaviour.  This module wires the fields
that carry behaviour to the state machine in :mod:`isaac_epics.motor`, and
leaves the rest served as inert-but-present fields so that clients which bind
them (Phoebus screens, ophyd) connect cleanly instead of timing out.

Everything here is glue.  Logic worth arguing about belongs in ``motor.py``,
where it can be tested without a Channel Access stack.
"""

from __future__ import annotations

import logging
from typing import Any, Dict, List, Optional

from caproto import ChannelType
from caproto.server import PVGroup, PvpropertyDouble, SubGroup, pvproperty

from .config import AdapterConfig, AxisConfig
from .motor import MotorAxis, Severity
from .plant import Plant
from .protocol import PROTOCOL_VERSION

logger = logging.getLogger(__name__)

#: caproto alarm severities, imported lazily to keep the mapping explicit.
_SEVERITY_MAP = {
    Severity.NO_ALARM: 0,
    Severity.MINOR: 1,
    Severity.MAJOR: 2,
    Severity.INVALID: 3,
}


class MotorPVGroup(PVGroup):
    """A single simulated motor served as a motor record.

    The main PV (no field suffix) is ``.VAL``; caproto attaches the full
    ``MotorFields`` group to it, so ``m1.RBV``, ``m1.DMOV`` and the rest exist
    automatically.  We additionally serve the behaviour-bearing fields as
    first-class pvproperties so that writes reach the state machine.
    """

    def __init__(self, *args: Any, axis: MotorAxis, **kwargs: Any) -> None:
        self.axis = axis
        self.cfg: AxisConfig = axis.config
        super().__init__(*args, **kwargs)

    # -- the value field ---------------------------------------------------

    val = pvproperty(
        value=0.0, name="", dtype=PvpropertyDouble, record="motor",
        doc="Desired value (.VAL) - writing starts a move",
        precision=4,
    )

    @val.putter
    async def val(self, instance: Any, value: float) -> float:
        """A write to ``.VAL`` starts a move."""
        accepted = self.axis.request_move(float(value))
        if not accepted:
            # Report the rejection rather than silently clamping: a scan that
            # records the position it asked for, having not gone there, is a
            # corrupted dataset.
            logger.warning(
                "%s: move to %g %s rejected (limits %g..%g, LVIO=%d)",
                self.cfg.motor, value, self.cfg.egu,
                self.axis.low_limit, self.axis.high_limit, 1,
            )
            await self._publish_lvio(1)
            # Return the live setpoint so .VAL reflects reality.
            return self.axis.setpoint
        await self._publish_lvio(0)
        return float(value)

    async def _publish_units(self) -> None:
        """Advertise engineering units on the value channels.

        Clients read units from channel metadata, not from ``.EGU``, so
        without this a plot axis or a progress bar is labelled with nothing
        -- and a millimetre is indistinguishable from a degree.
        """
        for prop in (self.val, self.rbv, self.rrbv, self.dval):
            try:
                await prop.write_metadata(units=self.cfg.egu)
            except Exception:  # noqa: BLE001
                logger.debug("could not set units on %r", prop, exc_info=True)

    async def _publish_limits(self) -> None:
        """Mirror .HLM/.LLM onto the .VAL channel's control limits.

        ophyd's ``EpicsMotor.limits`` reads the *control limits* of the .VAL
        channel, not the .HLM/.LLM fields, and so does Phoebus when it decides
        whether to grey out an entry box. Serving the fields without the
        channel metadata leaves every such client believing the axis has no
        limits at all -- it would happily offer to drive the twin somewhere
        the adapter is going to refuse.
        """
        try:
            await self.val.write_metadata(
                upper_ctrl_limit=self.axis.high_limit,
                lower_ctrl_limit=self.axis.low_limit,
            )
            await self.rbv.write_metadata(
                upper_ctrl_limit=self.axis.high_limit,
                lower_ctrl_limit=self.axis.low_limit,
            )
        except Exception:  # noqa: BLE001
            logger.debug("could not publish control limits", exc_info=True)

    async def _publish_precision(self) -> None:
        """Set display precision on every float field.

        Only ``.VAL`` carried precision, because it was declared on the
        pvproperty; every other double defaulted to 0. Clients honour that
        metadata, so a readback of 19.991924986 was rendered as ``20`` in a
        Bluesky LiveTable while the setpoint column showed four decimals.

        That is worse than untidy. The difference between commanded and
        measured is the quantity this whole project exists to expose -- it
        is how you see the deadband, the gravity sag, and whether a scan
        reached the position it recorded. Rounding it away in the default
        display hides the evidence.

        Engineering-unit fields get the axis's configured precision; times
        get three decimals, which is finer than any value they hold here.
        """
        egu_fields = (
            self.rbv, self.rrbv, self.dval, self.hlm, self.llm,
            self.dhlm, self.dllm, self.velo, self.vbas, self.vmax,
            self.bvel, self.rdbd, self.bdst, self.mres, self.twv,
            self.sim_target,
        )
        for prop in egu_fields:
            try:
                await prop.write_metadata(precision=self.cfg.precision)
            except Exception:  # noqa: BLE001 - metadata must not break startup
                logger.debug("could not set precision on %r", prop, exc_info=True)
        for prop in (self.accl, self.bacc, self.dly):
            try:
                await prop.write_metadata(precision=3)
            except Exception:  # noqa: BLE001
                logger.debug("could not set precision on %r", prop, exc_info=True)

    async def _publish_lvio(self, value: int) -> None:
        try:
            await self.lvio.write(value)
        except Exception:  # noqa: BLE001 - diagnostics must never break a move
            logger.debug("could not publish .LVIO", exc_info=True)

    # -- readback and motion status ---------------------------------------

    rbv = pvproperty(value=0.0, name=".RBV", dtype=PvpropertyDouble,
                     read_only=True, doc="Readback value")
    dmov = pvproperty(value=1, name=".DMOV", dtype=ChannelType.LONG,
                      read_only=True, doc="Done moving to value")
    movn = pvproperty(value=0, name=".MOVN", dtype=ChannelType.LONG,
                      read_only=True, doc="Motor is moving")
    rrbv = pvproperty(value=0.0, name=".DRBV", dtype=PvpropertyDouble,
                      read_only=True, doc="Dial readback value")
    dval = pvproperty(value=0.0, name=".DVAL", dtype=PvpropertyDouble,
                      read_only=True, doc="Dial desired value")
    tdir = pvproperty(value=1, name=".TDIR", dtype=ChannelType.LONG,
                      read_only=True, doc="Direction of travel")
    msta = pvproperty(value=0, name=".MSTA", dtype=ChannelType.LONG,
                      read_only=True, doc="Motor status word")
    miss = pvproperty(value=0, name=".MISS", dtype=ChannelType.LONG,
                      read_only=True, doc="Ran out of retries")
    rcnt = pvproperty(value=0, name=".RCNT", dtype=ChannelType.LONG,
                      read_only=True, doc="Retry count")
    lvio = pvproperty(value=0, name=".LVIO", dtype=ChannelType.LONG,
                      read_only=True, doc="Limit violation")
    hls = pvproperty(value=0, name=".HLS", dtype=ChannelType.LONG,
                     read_only=True, doc="At high limit switch")
    lls = pvproperty(value=0, name=".LLS", dtype=ChannelType.LONG,
                     read_only=True, doc="At low limit switch")
    athm = pvproperty(value=0, name=".ATHM", dtype=ChannelType.LONG,
                      read_only=True, doc="At home")

    # -- commands ----------------------------------------------------------

    stop_pv = pvproperty(value=0, name=".STOP", dtype=ChannelType.LONG,
                         doc="Stop the motor")

    @stop_pv.putter
    async def stop_pv(self, instance: Any, value: int) -> int:
        if value:
            logger.info("%s: .STOP", self.cfg.motor)
            self.axis.stop()
        return 0  # momentary

    homf = pvproperty(value=0, name=".HOMF", dtype=ChannelType.LONG,
                      doc="Home forward")

    @homf.putter
    async def homf(self, instance: Any, value: int) -> int:
        if value:
            self.axis.home(forward=True)
        return 0

    homr = pvproperty(value=0, name=".HOMR", dtype=ChannelType.LONG,
                      doc="Home reverse")

    @homr.putter
    async def homr(self, instance: Any, value: int) -> int:
        if value:
            self.axis.home(forward=False)
        return 0

    twv = pvproperty(value=1.0, name=".TWV", dtype=PvpropertyDouble,
                     doc="Tweak step size")
    twf = pvproperty(value=0, name=".TWF", dtype=ChannelType.LONG,
                     doc="Tweak forward")

    @twf.putter
    async def twf(self, instance: Any, value: int) -> int:
        if value:
            self.axis.request_relative(float(self.twv.value))
        return 0

    twr = pvproperty(value=0, name=".TWR", dtype=ChannelType.LONG,
                     doc="Tweak reverse")

    @twr.putter
    async def twr(self, instance: Any, value: int) -> int:
        if value:
            self.axis.request_relative(-float(self.twv.value))
        return 0

    # -- calibration fields ------------------------------------------------

    off = pvproperty(value=0.0, name=".OFF", dtype=PvpropertyDouble,
                     doc="User offset")

    @off.putter
    async def off(self, instance: Any, value: float) -> float:
        self.axis.offset = float(value)
        return float(value)

    dir_pv = pvproperty(value=0, name=".DIR", dtype=ChannelType.LONG,
                        doc="User direction (0=Pos, 1=Neg)")

    @dir_pv.putter
    async def dir_pv(self, instance: Any, value: int) -> int:
        self.axis.direction = 1 if int(value) else 0
        return self.axis.direction

    foff = pvproperty(value=0, name=".FOFF", dtype=ChannelType.LONG,
                      doc="Offset freeze switch (0=Variable, 1=Frozen)")

    @foff.putter
    async def foff(self, instance: Any, value: int) -> int:
        self.axis.freeze_offset = 1 if int(value) else 0
        return self.axis.freeze_offset

    set_pv = pvproperty(value=0, name=".SET", dtype=ChannelType.LONG,
                        doc="Set/Use switch (0=Use, 1=Set)")

    @set_pv.putter
    async def set_pv(self, instance: Any, value: int) -> int:
        self.axis.set_mode = 1 if int(value) else 0
        return self.axis.set_mode

    # -- motion parameters -------------------------------------------------

    velo = pvproperty(value=1.0, name=".VELO", dtype=PvpropertyDouble,
                      doc="Velocity (EGU/s)")

    @velo.putter
    async def velo(self, instance: Any, value: float) -> float:
        # Clamp to .VMAX exactly as the motor record does, rather than
        # accepting a value that would be silently ignored.
        clamped = max(1e-9, min(float(value), self.axis.max_velocity))
        self.axis.velocity = clamped
        return clamped

    vbas = pvproperty(value=0.0, name=".VBAS", dtype=PvpropertyDouble,
                      doc="Base velocity (EGU/s)")

    @vbas.putter
    async def vbas(self, instance: Any, value: float) -> float:
        self.axis.base_velocity = max(0.0, float(value))
        return self.axis.base_velocity

    vmax = pvproperty(value=1.0, name=".VMAX", dtype=PvpropertyDouble,
                      doc="Max velocity (EGU/s)")

    @vmax.putter
    async def vmax(self, instance: Any, value: float) -> float:
        self.axis.max_velocity = max(0.0, float(value))
        return self.axis.max_velocity

    accl = pvproperty(value=0.2, name=".ACCL", dtype=PvpropertyDouble,
                      doc="Seconds to velocity")

    @accl.putter
    async def accl(self, instance: Any, value: float) -> float:
        self.axis.accel_time = max(0.0, float(value))
        return self.axis.accel_time

    # -- limits ------------------------------------------------------------

    hlm = pvproperty(value=0.0, name=".HLM", dtype=PvpropertyDouble,
                     doc="User high limit")

    @hlm.putter
    async def hlm(self, instance: Any, value: float) -> float:
        if float(value) <= self.axis.low_limit:
            logger.warning("%s: rejected .HLM %g (not above .LLM %g)",
                           self.cfg.motor, value, self.axis.low_limit)
            return self.axis.high_limit
        self.axis.high_limit = float(value)
        await self._publish_limits()
        return float(value)

    llm = pvproperty(value=0.0, name=".LLM", dtype=PvpropertyDouble,
                     doc="User low limit")

    @llm.putter
    async def llm(self, instance: Any, value: float) -> float:
        if float(value) >= self.axis.high_limit:
            logger.warning("%s: rejected .LLM %g (not below .HLM %g)",
                           self.cfg.motor, value, self.axis.high_limit)
            return self.axis.low_limit
        self.axis.low_limit = float(value)
        await self._publish_limits()
        return float(value)

    dhlm = pvproperty(value=0.0, name=".DHLM", dtype=PvpropertyDouble,
                      read_only=True, doc="Dial high limit")
    dllm = pvproperty(value=0.0, name=".DLLM", dtype=PvpropertyDouble,
                      read_only=True, doc="Dial low limit")

    # -- deadband and retries ---------------------------------------------

    rdbd = pvproperty(value=1e-3, name=".RDBD", dtype=PvpropertyDouble,
                      doc="Retry deadband (EGU)")

    @rdbd.putter
    async def rdbd(self, instance: Any, value: float) -> float:
        self.axis.retry_deadband = max(1e-12, float(value))
        return self.axis.retry_deadband

    rtry = pvproperty(value=1, name=".RTRY", dtype=ChannelType.LONG,
                      doc="Max retry count")

    @rtry.putter
    async def rtry(self, instance: Any, value: int) -> int:
        self.axis.max_retries = max(0, int(value))
        return self.axis.max_retries

    bdst = pvproperty(value=0.0, name=".BDST", dtype=PvpropertyDouble,
                      doc="Backlash distance (EGU)")

    @bdst.putter
    async def bdst(self, instance: Any, value: float) -> float:
        self.axis.backlash_distance = float(value)
        return float(value)

    bvel = pvproperty(value=1.0, name=".BVEL", dtype=PvpropertyDouble,
                      doc="Backlash velocity (EGU/s)")

    @bvel.putter
    async def bvel(self, instance: Any, value: float) -> float:
        self.axis.backlash_velocity = max(1e-9, float(value))
        return self.axis.backlash_velocity

    bacc = pvproperty(value=0.2, name=".BACC", dtype=PvpropertyDouble,
                      doc="Backlash acceleration time")

    @bacc.putter
    async def bacc(self, instance: Any, value: float) -> float:
        self.axis.backlash_accel = max(0.0, float(value))
        return self.axis.backlash_accel

    dly = pvproperty(value=0.0, name=".DLY", dtype=PvpropertyDouble,
                     doc="Readback settle time (s)")

    @dly.putter
    async def dly(self, instance: Any, value: float) -> float:
        self.axis.settle_time = max(0.0, float(value))
        return self.axis.settle_time

    # -- descriptive -------------------------------------------------------

    egu = pvproperty(value="mm", name=".EGU", dtype=ChannelType.STRING,
                     doc="Engineering units")
    desc = pvproperty(value="", name=".DESC", dtype=ChannelType.STRING,
                      doc="Description")
    prec = pvproperty(value=4, name=".PREC", dtype=ChannelType.LONG,
                      doc="Display precision")
    mres = pvproperty(value=1e-4, name=".MRES", dtype=PvpropertyDouble,
                      doc="Motor step size (EGU)")

    # -- simulation-specific diagnostics ----------------------------------
    # Deliberately namespaced so they can never be mistaken for motor record
    # fields. A real motor has no ":SIM_" PVs; seeing one tells you instantly
    # that you are connected to the twin.

    sim_link = pvproperty(value="disconnected", name=":SIM_LINK",
                          dtype=ChannelType.STRING, read_only=True,
                          doc="Link state: disconnected / paused / running")
    sim_target = pvproperty(value=0.0, name=":SIM_TARGET", dtype=PvpropertyDouble,
                            read_only=True,
                            doc="Commanded joint position in simulation units")
    sim_joint = pvproperty(value="", name=":SIM_JOINT", dtype=ChannelType.STRING,
                           read_only=True, doc="Isaac Sim joint name")

    # -- startup -----------------------------------------------------------

    @val.startup
    async def val(self, instance: Any, async_lib: Any) -> None:
        """Seed static fields once the server is up."""
        await self.egu.write(self.cfg.egu)
        await self.desc.write(self.cfg.desc or self.cfg.motor)
        await self.prec.write(self.cfg.precision)
        await self.mres.write(self.cfg.resolution)
        await self.velo.write(self.cfg.velocity)
        await self.vmax.write(self.cfg.max_velocity)
        await self.vbas.write(self.cfg.base_velocity)
        await self.accl.write(self.cfg.accel_time)
        await self.hlm.write(self.cfg.high_limit)
        await self.llm.write(self.cfg.low_limit)
        await self.rdbd.write(self.cfg.retry_deadband)
        await self.rtry.write(self.cfg.max_retries)
        await self.bdst.write(self.cfg.backlash_distance)
        await self.bvel.write(self.cfg.backlash_velocity)
        await self.dly.write(self.cfg.settle_time)
        await self.twv.write(max(self.cfg.retry_deadband * 10, 0.1))
        await self.sim_joint.write(self.cfg.joint)
        await self.dir_pv.write(0 if self.cfg.direction == "Pos" else 1)
        await self._publish_limits()
        await self._publish_precision()
        await self._publish_units()

    # -- publication -------------------------------------------------------

    async def publish(self, outputs: Any) -> None:
        """Push one tick's outputs onto Channel Access.

        Values are written unconditionally rather than only on change:
        caproto suppresses duplicate monitor updates itself, and gating here
        would risk a client missing a transition.
        """
        sev = _SEVERITY_MAP[outputs.severity]
        await _write(self.rbv, outputs.readback, severity=sev)
        await _write(self.rrbv, outputs.dial_readback)
        await _write(self.dval, outputs.dial_setpoint)
        await _write(self.dmov, outputs.done_moving, severity=sev)
        await _write(self.movn, outputs.moving)
        await _write(self.tdir, outputs.direction_of_travel)
        await _write(self.msta, outputs.status)
        await _write(self.miss, outputs.missed)
        await _write(self.rcnt, outputs.retry_count)
        await _write(self.lvio, outputs.limit_violation)
        await _write(self.hls, outputs.high_limit_switch)
        await _write(self.lls, outputs.low_limit_switch)
        await _write(self.athm, outputs.at_home)
        await _write(self.dhlm, self.axis.dial_high_limit)
        await _write(self.dllm, self.axis.dial_low_limit)
        await _write(self.sim_link, outputs.link_state)
        if outputs.target_sim is not None:
            await _write(self.sim_target, outputs.target_sim)
        # .VAL tracks the state machine so that a .SET-mode redefinition or a
        # .STOP is reflected back to clients.
        if abs(float(self.val.value) - self.axis.setpoint) > 1e-12:
            await _write(self.val, self.axis.setpoint)


class StatusPVGroup(PVGroup):
    """Adapter-wide health PVs.

    Section 8 requires that disconnected / paused / running be distinguishable
    and surfaced.  Per-motor alarms carry that for each axis; these PVs carry
    it for the adapter as a whole, which is what an overview screen and an
    alarm handler want to watch.
    """

    link = pvproperty(value="disconnected", name="LINK", dtype=ChannelType.STRING,
                      read_only=True, doc="disconnected / paused / running")
    connected = pvproperty(value=0, name="CONNECTED", dtype=ChannelType.LONG,
                           read_only=True, doc="1 when Isaac Sim is posting frames")
    playing = pvproperty(value=0, name="PLAYING", dtype=ChannelType.LONG,
                         read_only=True, doc="1 when the simulation is stepping")
    age = pvproperty(value=0.0, name="AGE", dtype=PvpropertyDouble, read_only=True,
                     doc="Seconds since the last frame from Isaac Sim")
    t_sim = pvproperty(value=0.0, name="TSIM", dtype=PvpropertyDouble, read_only=True,
                       doc="Simulation clock (s)")
    frames = pvproperty(value=0, name="FRAMES", dtype=ChannelType.LONG, read_only=True,
                        doc="Frames accepted since adapter start")
    protocol = pvproperty(value=PROTOCOL_VERSION, name="PROTO", dtype=ChannelType.LONG,
                          read_only=True, doc="Wire protocol version")
    cfg_ok = pvproperty(value=-1, name="CFGOK", dtype=ChannelType.LONG, read_only=True,
                        doc="Config vs articulation: 1 ok, 0 errors, -1 not yet checked")
    cfg_msg = pvproperty(value="not checked", name="CFGMSG", dtype=ChannelType.STRING,
                         read_only=True, doc="First configuration finding, if any")

    async def publish_check(self, findings: Any) -> None:
        """Publish the verdict of the configuration check."""
        bad = [f for f in findings if getattr(f, "severity", "") == "error"]
        await _write(self.cfg_ok, 0 if bad else 1, severity=2 if bad else 0)
        if findings:
            first = str(findings[0])
            await _write(self.cfg_msg, first[:39], severity=2 if bad else 1)
        else:
            await _write(self.cfg_msg, "ok")

    async def publish(self, state: Any) -> None:
        sev = 0 if state.stepping else (3 if not state.connected else 1)
        await _write(self.link, state.link_state, severity=sev)
        await _write(self.connected, 1 if state.connected else 0, severity=sev)
        await _write(self.playing, 1 if state.playing else 0)
        await _write(self.age, min(state.age, 1e6) if state.age != float("inf") else 1e6,
                     severity=sev)
        await _write(self.t_sim, state.t_sim)
        await _write(self.frames, state.frames)


async def _write(prop: Any, value: Any, severity: Optional[int] = None) -> None:
    """Write a pvproperty, tolerating a value the channel cannot represent."""
    try:
        if severity is None:
            await prop.write(value)
        else:
            await prop.write(value, severity=severity)
    except Exception:  # noqa: BLE001 - one bad field must not stall the loop
        logger.debug("failed to publish %r = %r", getattr(prop, "name", prop), value,
                     exc_info=True)


def build_ioc(config: AdapterConfig, plant: Plant) -> "AdapterIOC":
    """Construct the top-level PVGroup for every configured axis."""
    axes = {cfg.motor: MotorAxis(cfg) for cfg in config.axes}
    members: Dict[str, Any] = {
        "_axes": axes,
        "_config": config,
        "_plant": plant,
        "status": SubGroup(StatusPVGroup, prefix=""),
    }
    for cfg in config.axes:
        members[f"motor_{cfg.motor}"] = SubGroup(
            MotorPVGroup, prefix=cfg.motor, axis=axes[cfg.motor],
        )
    cls = type("AdapterIOC", (AdapterIOC,), members)
    return cls(prefix=config.prefix)


class AdapterIOC(PVGroup):
    """Base class for the generated top-level group.

    The concrete subclass is built by :func:`build_ioc` because the set of
    motors is configuration-driven and ``SubGroup`` members must exist at
    class definition time.
    """

    _axes: Dict[str, MotorAxis] = {}
    _config: Optional[AdapterConfig] = None
    _plant: Optional[Plant] = None

    @property
    def axes(self) -> Dict[str, MotorAxis]:
        return self._axes

    def motor_group(self, name: str) -> MotorPVGroup:
        return getattr(self, f"motor_{name}")

    def diagnostics(self) -> Dict[str, Any]:
        """Per-axis summary echoed to the client and served on ``/state``."""
        out: Dict[str, Any] = {}
        for name, axis in self._axes.items():
            out[name] = {
                "joint": axis.config.joint,
                "egu": axis.config.egu,
                "setpoint": round(axis.setpoint, 9),
                "readback": round(axis.readback, 9),
                "phase": axis.phase.value,
                "moving": axis.in_motion,
            }
        return out

    async def tick_all(self) -> None:
        """Run every axis once and publish the results."""
        assert self._plant is not None
        state = self._plant.get_state()
        targets: Dict[str, float] = {}
        for name, axis in self._axes.items():
            outputs = axis.tick(state)
            if outputs.target_sim is not None:
                targets[axis.config.joint] = outputs.target_sim
            await self.motor_group(name).publish(outputs)
        if targets:
            self._plant.set_targets(targets)
        await self.status.publish(state)


def axis_names(config: AdapterConfig) -> List[str]:
    return [a.motor for a in config.axes]
