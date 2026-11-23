"""Check the adapter's configuration against the articulation that exists.

Most of the per-model work when someone brings their own CAD is not writing
code -- it is keeping a JSON file in step with a mechanism authored
elsewhere. That is exactly the kind of agreement that rots silently:

* a ``joint`` name that matches nothing drives nothing, and the axis simply
  never moves while every PV looks healthy;
* a soft limit outside the physical joint lets a client command a position
  the mechanism cannot reach, so the move stalls against the PhysX limit and
  reports ``.MISS`` after retries;
* a stage authored in millimetres against a config written for metres is
  wrong by a factor of a thousand, which looks like a wild mechanism rather
  than a unit error.

All three present as faults rather than as the configuration errors they
are. The simulation knows the truth and can simply say so, so the adapter
checks on the first frame and reports what it finds.

This module is pure: it takes a config and a descriptor and returns
findings. Nothing here does I/O or touches motor state.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional

from .config import AdapterConfig
from .protocol import Articulation


@dataclass(frozen=True)
class Finding:
    """One disagreement between the configuration and the articulation."""

    severity: str   # "error" | "warning"
    motor: str
    message: str

    def __str__(self) -> str:
        where = f"{self.motor}: " if self.motor else ""
        return f"{self.severity.upper()}: {where}{self.message}"


def _check_declared_bindings(config: AdapterConfig, articulation: Articulation,
                             findings: List[Finding]) -> None:
    """Compare PV bindings the MODEL declares against what the config serves.

    A joint prim may carry ``epics:motorPV``, naming the PV it expects to be
    driven by. When it does, there are two statements of the same fact -- one
    in the asset, one in the config -- and they can disagree. Saying so is the
    whole value of putting the binding in the model: a joint name typed into
    a config file is an agreement nobody re-reads, while a declared binding
    is checked on the first frame of every run.

    A stage that declares nothing is not a problem and is not reported; that
    is every stage authored before this existed.
    """
    declared = {j.name: j.motor_pv for j in articulation.joints if j.motor_pv}
    if not declared:
        return

    served = {axis.joint: config.prefix + axis.motor for axis in config.axes}

    for joint, want_pv in sorted(declared.items()):
        have_pv = served.get(joint)
        if have_pv is None:
            findings.append(Finding(
                "warning", "",
                f"the model declares joint {joint!r} should be driven by "
                f"{want_pv!r}, but no configured axis drives that joint. "
                f"The asset expects a motor this adapter is not serving."
            ))
        elif have_pv != want_pv:
            findings.append(Finding(
                "error", "",
                f"joint {joint!r} is declared in the model as {want_pv!r} but "
                f"is served here as {have_pv!r}. One of the two is wrong, and "
                f"a scan written against the declared name would drive "
                f"nothing while every PV looked healthy."
            ))


def check(config: AdapterConfig, articulation: Optional[Articulation]) -> List[Finding]:
    """Compare ``config`` against what the simulation reports.

    Returns findings worst-first. An empty list means the configuration
    matches the mechanism on every point that can be checked.
    """
    findings: List[Finding] = []

    if articulation is None:
        findings.append(Finding(
            "warning", "",
            "the simulation did not describe its articulation, so the "
            "configuration could not be checked against it. An older client, "
            "or one that failed to introspect the stage.",
        ))
        return findings

    available = [j.name for j in articulation.joints]

    if articulation.meters_per_unit and not math.isclose(
            articulation.meters_per_unit, 1.0, rel_tol=1e-9):
        findings.append(Finding(
            "error", "",
            f"the stage is authored at {articulation.meters_per_unit} metres per "
            f"unit, but the wire protocol is defined in metres and radians. "
            f"Positions will be wrong by a factor of "
            f"{1.0 / articulation.meters_per_unit:g}. Re-export the asset with "
            f"metersPerUnit = 1.0, or scale it on import.",
        ))

    for axis in config.axes:
        info = articulation.joint(axis.joint)
        if info is None:
            findings.append(Finding(
                "error", axis.motor,
                f"joint {axis.joint!r} does not exist in the articulation, so "
                f"this motor will never move. Available joints: "
                f"{', '.join(available) if available else '(none reported)'}",
            ))
            continue

        # Joint type against the unit mapping. A revolute joint served in mm,
        # or a prismatic one served in degrees, is a conversion error that
        # produces plausible-looking numbers.
        rotary_egu = axis.egu.lower() in ("deg", "degrees", "rad", "mrad")
        if info.joint_type == "revolute" and not rotary_egu:
            findings.append(Finding(
                "warning", axis.motor,
                f"joint {axis.joint!r} is revolute but is served in "
                f"{axis.egu!r}. Rotary joints report radians.",
            ))
        elif info.joint_type == "prismatic" and rotary_egu:
            findings.append(Finding(
                "warning", axis.motor,
                f"joint {axis.joint!r} is prismatic but is served in "
                f"{axis.egu!r}. Linear joints report metres.",
            ))

        if info.lower is None or info.upper is None:
            continue

        low_sim = axis.to_sim(axis.low_limit)
        high_sim = axis.to_sim(axis.high_limit)
        # .DIR = Neg inverts the mapping, so compare the span, not the ends.
        lo, hi = min(low_sim, high_sim), max(low_sim, high_sim)
        tol = abs(info.upper - info.lower) * 1e-9

        if lo < info.lower - tol:
            findings.append(Finding(
                "error", axis.motor,
                f"soft low limit {axis.low_limit:g} {axis.egu} is outside the "
                f"{axis.joint} joint limit ({info.lower:g} in simulation units, "
                f"= {axis.to_egu(info.lower):g} {axis.egu}). Moves below the "
                f"joint limit will stall and report .MISS.",
            ))
        if hi > info.upper + tol:
            findings.append(Finding(
                "error", axis.motor,
                f"soft high limit {axis.high_limit:g} {axis.egu} is outside the "
                f"{axis.joint} joint limit ({info.upper:g} in simulation units, "
                f"= {axis.to_egu(info.upper):g} {axis.egu}). Moves beyond the "
                f"joint limit will stall and report .MISS.",
            ))

    unused = [name for name in available
              if name not in {a.joint for a in config.axes}]
    if unused:
        findings.append(Finding(
            "warning", "",
            f"the articulation has joints no motor drives: {', '.join(unused)}. "
            f"They will hold their position.",
        ))

    _check_declared_bindings(config, articulation, findings)

    order = {"error": 0, "warning": 1}
    return sorted(findings, key=lambda f: order.get(f.severity, 2))


def errors(findings: List[Finding]) -> List[Finding]:
    return [f for f in findings if f.severity == "error"]
