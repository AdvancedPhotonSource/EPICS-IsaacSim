"""Confirm the adapter's field coverage against the installed ophyd.

Section 9 of the design document lists "confirm the ophyd EpicsMotor field
list" as an open question, noting the list there was model knowledge rather
than verified. This test answers it against whatever ophyd is actually
installed, so the answer cannot drift.
"""

import pytest

from isaac_epics.config import default_config
from isaac_epics.ioc import build_ioc
from isaac_epics.plant import SharedPlant

ophyd = pytest.importorskip("ophyd", reason="ophyd not installed")


def test_adapter_serves_every_field_ophyd_binds():
    from ophyd import EpicsMotor

    suffixes = sorted({
        cpt.suffix for cpt in EpicsMotor._sig_attrs.values()
        if getattr(cpt, "suffix", None)
    })
    assert suffixes, "could not introspect EpicsMotor"

    pvdb = build_ioc(default_config(), SharedPlant()).pvdb
    missing = []
    for suffix in suffixes:
        name = "SIM:TWIN:m1" if suffix == ".VAL" else f"SIM:TWIN:m1{suffix}"
        if name not in pvdb:
            missing.append(suffix)
    assert not missing, (
        f"ophyd {ophyd.__version__} binds fields this adapter does not serve: {missing}"
    )
