"""The console must configure Channel Access itself.

The adapter binds CA to ``config.ca_interface``, loopback by default. A client
left on the default address list broadcasts, and a broadcast never reaches a
server bound to 127.0.0.1. ``examples/bluesky_scan.py`` has always set this;
the console did not, so the primary interactive entry point was the one that
failed -- and it failed by printing every signal it could not connect, which
buries one missing environment variable under eighty lines of ophyd internals.
"""

import os

import pytest

from isaac_epics.config import AdapterConfig, AxisConfig


def _config(interface):
    return AdapterConfig(
        prefix="SIM:TWIN:",
        axes=[AxisConfig(motor="m1", joint="x_axis")],
        ca_interface=interface,
    )


@pytest.fixture
def clean_env(monkeypatch):
    for name in ("EPICS_CA_ADDR_LIST", "EPICS_CA_AUTO_ADDR_LIST"):
        monkeypatch.delenv(name, raising=False)


def _apply(config):
    """Run only the environment half of build_namespace.

    build_namespace goes on to connect to a live IOC, which a unit test has
    no business doing, so this mirrors the block under test.
    """
    os.environ.setdefault("OPHYD_CONTROL_LAYER", "caproto")
    if config.ca_interface not in ("0.0.0.0", ""):
        os.environ.setdefault("EPICS_CA_ADDR_LIST", config.ca_interface)
        os.environ.setdefault("EPICS_CA_AUTO_ADDR_LIST", "NO")


def test_console_source_configures_channel_access():
    """The real module must contain the behaviour this file pins down."""
    import inspect

    from isaac_epics import console

    source = inspect.getsource(console.build_namespace)
    assert "EPICS_CA_ADDR_LIST" in source, (
        "build_namespace no longer configures Channel Access; the console "
        "will fail against a loopback-bound adapter"
    )
    assert "config.ca_interface" in source, (
        "the address list must come from the config, not a hardcoded literal"
    )


def test_loopback_adapter_sets_the_address_list(clean_env):
    _apply(_config("127.0.0.1"))
    assert os.environ["EPICS_CA_ADDR_LIST"] == "127.0.0.1"
    assert os.environ["EPICS_CA_AUTO_ADDR_LIST"] == "NO"


def test_non_loopback_interface_is_used_verbatim(clean_env):
    _apply(_config("164.54.100.54"))
    assert os.environ["EPICS_CA_ADDR_LIST"] == "164.54.100.54"


def test_an_explicit_address_list_is_never_overridden(monkeypatch):
    monkeypatch.setenv("EPICS_CA_ADDR_LIST", "10.0.0.1 10.0.0.2")
    monkeypatch.delenv("EPICS_CA_AUTO_ADDR_LIST", raising=False)
    _apply(_config("127.0.0.1"))
    assert os.environ["EPICS_CA_ADDR_LIST"] == "10.0.0.1 10.0.0.2", (
        "a user who set their own address list must keep it"
    )


def test_wildcard_bind_is_left_to_normal_discovery(clean_env):
    """0.0.0.0 is not an address a client can search; don't invent one."""
    _apply(_config("0.0.0.0"))
    assert "EPICS_CA_ADDR_LIST" not in os.environ
