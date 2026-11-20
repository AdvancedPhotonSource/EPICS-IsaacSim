"""ophyd device definitions, generated from the adapter configuration.

The device and the IOC are built from the same ``AdapterConfig``, so the
console cannot drift out of step with what is actually being served. Add an
axis to the config and it appears in both.

Nothing here is simulation-aware except one signal. The motors are stock
``EpicsMotor`` components, which is the whole point of section 5 of the design
document: the twin stands in for the hardware, so the device definition that
drives it must be the same one that would drive the beamline. A device class
with twin-specific behaviour would be a reimplementation, and testing a
reimplementation tells you nothing about the real scan.

The exception is :attr:`link`, which is deliberately *outside* the motors and
namespaced so it can never be mistaken for a motor record field. Checking it
before a run is how you tell a live simulation from a paused or vanished one.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

from .config import AdapterConfig


def make_stage_class(config: AdapterConfig, name: str = "SimStage") -> type:
    """Build an ophyd ``Device`` class exposing every configured axis.

    Args:
        config: The adapter configuration being served.
        name: Class name, for nicer reprs.

    Returns:
        A new ``Device`` subclass with one ``EpicsMotor`` per axis, attribute
        named by each axis's ``alias``, plus a read-only ``link`` signal.

    Raises:
        ImportError: if ophyd is not installed. It is an optional dependency;
            the adapter itself does not need it.
    """
    try:
        from ophyd import Component as Cpt
        from ophyd import Device, EpicsMotor, EpicsSignalRO
    except ImportError as exc:  # pragma: no cover - optional dependency
        raise ImportError(
            "ophyd is required for the console and device definitions. "
            "Install it with: pip install 'isaac-epics[bluesky]'"
        ) from exc

    members: Dict[str, Any] = {
        "__doc__": (
            f"Simulated stage served at {config.prefix}.\n\n"
            "Stock EpicsMotor components -- nothing here knows it is a twin."
        ),
    }
    for axis in config.axes:
        members[axis.console_name] = Cpt(
            EpicsMotor, axis.motor, kind="hinted",
            doc=f"{axis.desc or axis.motor} ({axis.egu}) -> joint {axis.joint}",
        )
    members["link"] = Cpt(
        EpicsSignalRO, "LINK", string=True, kind="config",
        doc="disconnected / paused / running",
    )
    return type(name, (Device,), members)


def connect_stage(
    config: AdapterConfig,
    timeout: float = 20.0,
    name: str = "stage",
) -> Any:
    """Instantiate and connect the stage device.

    Raises:
        TimeoutError: with a message naming the prefix, since a failure here
            is almost always "the adapter is not running" or "the PV prefix
            does not match", and both are worth saying out loud.
    """
    cls = make_stage_class(config)
    stage = cls(config.prefix, name=name)
    try:
        stage.wait_for_connection(timeout=timeout)
    except Exception as exc:
        raise TimeoutError(
            f"could not connect to {config.prefix}* within {timeout:g}s: {exc}\n"
            f"Is the adapter running?  isaac-epics-ioc -c <config>\n"
            f"Is EPICS_CA_ADDR_LIST set to reach it?"
        ) from exc
    return stage


def axis_table(config: AdapterConfig, stage: Optional[Any] = None) -> str:
    """Render a human-readable table of the configured axes.

    With ``stage`` connected, includes live readbacks -- which makes it the
    fastest way to see both what is configured and where things actually are.
    """
    rows: List[str] = []
    head = "{:<8} {:<6} {:<12} {:>10} {:>10} {:>6}".format(
        "name", "pv", "joint", "low", "high", "egu")
    if stage is not None:
        head += " {:>12}".format("readback")
    rows.append(head)
    rows.append("-" * len(head))
    for axis in config.axes:
        row = f"{axis.console_name:<8} {axis.motor:<6} {axis.joint:<12} {axis.low_limit:>10.3f} {axis.high_limit:>10.3f} {axis.egu:>6}"
        if stage is not None:
            try:
                row += f" {getattr(stage, axis.console_name).position:>12.4f}"
            except Exception:  # noqa: BLE001 - a dead PV must not break the table
                row += " {:>12}".format("??")
        rows.append(row)
    return "\n".join(rows)
