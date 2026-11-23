"""Channel Access layer: PV surface and metadata."""

import pytest

from isaac_epics.config import AdapterConfig, AxisConfig, default_config
from isaac_epics.ioc import build_ioc
from isaac_epics.plant import SharedPlant


@pytest.fixture
def ioc():
    return build_ioc(default_config(), SharedPlant())


def test_every_ophyd_epicsmotor_field_is_served(ioc):
    """ophyd 1.11.2 binds these 19 signals; all must exist as live PVs.

    Verified against the installed ophyd in test_ophyd_fields below; this list
    is the frozen expectation so a regression is caught without ophyd.
    """
    required = [".RBV", ".VAL", ".OFF", ".DIR", ".FOFF", ".SET", ".VELO",
                ".ACCL", ".EGU", ".MOVN", ".DMOV", ".HLS", ".LLS", ".HLM",
                ".LLM", ".TDIR", ".STOP", ".HOMF", ".HOMR"]
    pvdb = ioc.pvdb
    for field in required:
        name = "SIM:TWIN:m1" if field == ".VAL" else f"SIM:TWIN:m1{field}"
        assert name in pvdb, f"{field} is not served"


def test_real_beamline_fields_are_served(ioc):
    for field in (".MSTA", ".LVIO", ".MRES", ".BDST", ".RTRY", ".RDBD",
                  ".MISS", ".RCNT", ".DHLM", ".DLLM", ".BVEL", ".DLY"):
        assert f"SIM:TWIN:m1{field}" in ioc.pvdb, f"{field} is not served"


def test_caproto_supplies_the_full_motor_record(ioc):
    """The 153 generated fields back everything we do not serve explicitly."""
    base = ioc.pvdb["SIM:TWIN:m1"]
    assert base.field_inst is not None
    assert len(base.field_inst.pvdb) == 153


def test_explicit_pvs_shadow_the_generated_fields(ioc):
    """Our live .RBV must win over the inert generated one."""
    assert "SIM:TWIN:m1.RBV" in ioc.pvdb, (
        "caproto resolves pvdb before record fields, so the live value only "
        "wins if it is present as its own entry"
    )


def test_simulation_pvs_are_namespaced(ioc):
    """A real motor has no ':SIM_' PVs - that is how you tell them apart."""
    for suffix in (":SIM_LINK", ":SIM_TARGET", ":SIM_JOINT"):
        assert f"SIM:TWIN:m1{suffix}" in ioc.pvdb
    for name in ioc.pvdb:
        if "SIM_" in name:
            assert ":SIM_" in name, f"{name} could be mistaken for a record field"


def test_adapter_health_pvs_exist(ioc):
    for suffix in ("LINK", "CONNECTED", "PLAYING", "AGE", "TSIM", "FRAMES", "PROTO"):
        assert f"SIM:TWIN:{suffix}" in ioc.pvdb


def test_each_axis_gets_its_own_pvs(ioc):
    for motor in ("m1", "m2", "m3", "m4"):
        assert f"SIM:TWIN:{motor}" in ioc.pvdb


def test_diagnostics_are_json_serialisable(ioc):
    import json
    json.dumps(ioc.diagnostics())
    assert set(ioc.diagnostics()) == {"m1", "m2", "m3", "m4"}


async def test_control_limits_are_published(ioc):
    """ophyd's EpicsMotor.limits reads channel control limits, not .HLM/.LLM.

    Serving the fields but not the metadata leaves ophyd and Phoebus believing
    the axis is unlimited, and they will offer to drive it somewhere the
    adapter is going to refuse.
    """
    group = ioc.motor_group("m1")
    await group._publish_limits()
    assert group.val.upper_ctrl_limit == pytest.approx(24.0)
    assert group.val.lower_ctrl_limit == pytest.approx(-24.0)
    assert group.rbv.upper_ctrl_limit == pytest.approx(24.0)


async def test_changing_hlm_updates_the_control_limits(ioc):
    group = ioc.motor_group("m1")
    await group._publish_limits()
    await group.hlm.write(20.0)
    assert group.axis.high_limit == pytest.approx(20.0)
    assert group.val.upper_ctrl_limit == pytest.approx(20.0)


async def test_inverted_limit_write_is_refused(ioc):
    group = ioc.motor_group("m1")
    await group.hlm.write(-100.0)       # below .LLM
    assert group.axis.high_limit == pytest.approx(24.0), "limits must stay ordered"


async def test_velo_is_clamped_to_vmax(ioc):
    group = ioc.motor_group("m1")
    await group.vmax.write(20.0)
    await group.velo.write(999.0)
    assert group.axis.velocity == pytest.approx(20.0)


def test_prefix_is_applied_everywhere():
    cfg = AdapterConfig(prefix="SIM:OTHER:",
                        axes=[AxisConfig(motor="mx", joint="jx")])
    pvdb = build_ioc(cfg, SharedPlant()).pvdb
    assert all(name.startswith("SIM:OTHER:") for name in pvdb)


async def test_every_float_field_has_display_precision(ioc):
    """A readback rendered as an integer hides the measurement.

    Regression: only .VAL carried precision, because it was declared on the
    pvproperty. Everything else defaulted to 0, so a Bluesky LiveTable
    showed a readback of 19.991924986 as "20" next to a setpoint column of
    "-20.0000". The commanded-vs-measured difference is the quantity this
    project exists to expose; rounding it away by default hides the
    deadband, the gravity sag, and whether a scan reached what it recorded.
    """
    group = ioc.motor_group("m1")
    await group._publish_precision()
    for name in ("rbv", "rrbv", "dval", "hlm", "llm", "velo", "rdbd", "sim_target"):
        prop = getattr(group, name)
        assert prop.precision == group.cfg.precision, (
            f".{name} has precision {prop.precision}, expected {group.cfg.precision}"
        )


async def test_rotary_axis_gets_its_own_precision(ioc):
    """Precision is per-axis, not global: theta is configured coarser."""
    theta = ioc.motor_group("m4")
    await theta._publish_precision()
    assert theta.rbv.precision == theta.cfg.precision == 3


async def test_value_channels_advertise_engineering_units(ioc):
    """Clients read units from channel metadata, not from .EGU."""
    for motor, egu in (("m1", "mm"), ("m4", "deg")):
        group = ioc.motor_group(motor)
        await group._publish_units()
        assert group.rbv.units in (egu, egu.encode()), (
            f"{motor}.RBV units = {group.rbv.units!r}, expected {egu!r}"
        )


async def test_publish_path_actually_lands_every_field():
    """Tick the IOC for real and assert the published fields move.

    Everything above tests the PV *surface* -- that names, types and metadata
    exist. None of it ticks, so none of it covers ``MotorPVGroup.publish``,
    which is the code that puts values onto Channel Access.

    That matters because ``_write`` swallows every exception and logs it at
    DEBUG. A field that could not be written -- renamed, retyped, or given a
    value its channel cannot represent -- fails silently and forever, and the
    PV simply keeps returning its default. The adapter looks healthy and
    serves a lie, which is the one failure mode this project is built to
    refuse. So: drive a complete move and assert both that the values arrive
    and that nothing was swallowed on the way.
    """
    import logging

    from isaac_epics.plant import SharedPlant
    from isaac_epics.protocol import SimFrame

    swallowed = []

    class Catch(logging.Handler):
        def emit(self, record):
            swallowed.append(record.getMessage())

    handler = Catch()
    logger = logging.getLogger("isaac_epics.ioc")
    logger.addHandler(handler)
    previous = logger.level
    logger.setLevel(logging.DEBUG)

    try:
        plant = SharedPlant(stale_timeout=100.0)
        ioc = build_ioc(default_config(), plant)
        group = ioc.motor_m1
        joint = group.cfg.joint

        t_sim = 0.0
        position = 0.0
        plant.submit_frame(SimFrame(positions={joint: position}, t_sim=t_sim,
                                    playing=True, seq=1))
        await ioc.tick_all()

        await group.val.write(3.0)
        for seq in range(2, 400):
            targets = plant.targets
            if joint in targets:
                # Converge on whatever the trajectory asked for.
                position += (targets[joint] - position) * 0.5
            t_sim += 1 / 60.0
            plant.submit_frame(SimFrame(positions={joint: position}, t_sim=t_sim,
                                        playing=True, seq=seq))
            await ioc.tick_all()
            if group.dmov.value == 1 and seq > 10:
                break

        assert not swallowed, f"publish swallowed write failures: {swallowed}"

        # The readback reached the commanded position, in engineering units...
        assert group.rbv.value == pytest.approx(3.0, abs=group.cfg.retry_deadband)
        # ...and the completion flags agree with it.
        assert group.dmov.value == 1, "move never reported done"
        assert group.movn.value == 0
        assert group.lvio.value == 0
        assert group.sim_link.value == "running"
        assert group.sim_target.value == pytest.approx(
            group.cfg.to_sim(3.0), abs=1e-6)
        # Status group published too.
        assert ioc.status.link.value == "running"
        assert ioc.status.frames.value > 0
    finally:
        logger.removeHandler(handler)
        logger.setLevel(previous)
