"""The Isaac Sim half must ship with the adapter, and must stay standalone.

Two separate guarantees are tested here, and they fail in opposite
directions.

*Shipping.* ``epics_tick.py`` used to live outside the packaged tree, so
``pip install isaac-epics`` produced an adapter with no way to reach a
simulator: the file it needs was only in the git repository. These tests pin
the file as a package resource, which is what puts it in the wheel, and cover
``isaac-epics-tick`` as the supported way to get it back out.

*Staying standalone.* Now that the file sits inside a package whose other
modules import caproto, the stdlib-only rule in architecture decision 7.6 is
one convenient-looking import away from being broken. Nothing would fail
locally: the adapter's own interpreter has those packages. It would fail in
Isaac Sim's embedded interpreter, at a customer site, as an ImportError in
the per-frame tick. So the constraint is enforced statically, the same way
``test_mirror.py`` enforces that mirror mode cannot write to a PV.
"""

import ast
import os
import sys

import pytest

from isaac_epics.tickfile import TICK_FILENAME, install, main, tick_path, tick_source

#: Stdlib modules ``epics_tick`` is permitted to import, used only on Python
#: 3.9 where ``sys.stdlib_module_names`` does not exist. The adapter targets
#: 3.9 because that is the stock interpreter on the APS hosts.
_STDLIB_FALLBACK = frozenset({
    "json", "threading", "time", "http", "urllib", "socket", "sys", "os",
    "errno", "contextlib", "collections", "logging", "queue", "typing",
})


def _stdlib_names():
    return getattr(sys, "stdlib_module_names", None) or _STDLIB_FALLBACK


# -- shipping ---------------------------------------------------------------

def test_packaged_file_is_a_resolvable_resource():
    """The property that actually puts the file in the wheel."""
    path = tick_path()
    assert os.path.isfile(path)
    assert os.path.basename(path) == TICK_FILENAME


def test_source_is_the_real_client():
    source = tick_source()
    assert b"class EpicsTickClient" in source
    assert b"def exchange" in source


def test_install_writes_byte_identical_copy(tmp_path):
    assert install(str(tmp_path)) == 0
    written = (tmp_path / TICK_FILENAME).read_bytes()
    assert written == tick_source()


def test_installed_copy_is_importable_and_standalone(tmp_path, monkeypatch):
    """The emitted file works where it lands, with nothing else present.

    This is the whole promise of the command: drop it next to a standalone
    script and import it. If it needed anything from ``isaac_epics`` it would
    import here (the adapter is installed) and fail inside Isaac Sim.
    """
    install(str(tmp_path))
    monkeypatch.syspath_prepend(str(tmp_path))
    for name in ("epics_tick",):
        sys.modules.pop(name, None)
    module = __import__("epics_tick")
    try:
        assert hasattr(module, "EpicsTickClient")
        assert module.__file__.startswith(str(tmp_path))
    finally:
        sys.modules.pop("epics_tick", None)


def test_reinstall_is_a_noop(tmp_path, capsys):
    install(str(tmp_path))
    capsys.readouterr()
    assert install(str(tmp_path)) == 0
    assert "already up to date" in capsys.readouterr().out


def test_install_refuses_to_clobber_a_modified_copy(tmp_path, capsys):
    target = tmp_path / TICK_FILENAME
    target.write_bytes(b"# someone edited this\n")

    assert install(str(tmp_path)) == 1
    assert target.read_bytes() == b"# someone edited this\n"
    assert "--force" in capsys.readouterr().err


def test_force_overwrites_a_modified_copy(tmp_path):
    target = tmp_path / TICK_FILENAME
    target.write_bytes(b"# someone edited this\n")

    assert install(str(tmp_path), force=True) == 0
    assert target.read_bytes() == tick_source()


def test_install_rejects_a_path_that_is_not_a_directory(tmp_path, capsys):
    missing = tmp_path / "nope"
    assert install(str(missing)) == 2
    assert "not a directory" in capsys.readouterr().err


def test_cli_print_emits_exact_bytes(capsysbinary):
    assert main(["--print"]) == 0
    assert capsysbinary.readouterr().out == tick_source()


def test_cli_path_prints_the_location(capsys):
    assert main(["--path"]) == 0
    assert capsys.readouterr().out.strip() == tick_path()


def test_cli_requires_an_action():
    with pytest.raises(SystemExit):
        main([])


# -- staying standalone -----------------------------------------------------

def test_tick_client_imports_only_the_standard_library():
    tree = ast.parse(tick_source().decode("utf-8"), filename=TICK_FILENAME)
    stdlib = _stdlib_names()

    offenders = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names = [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom):
            # A relative import would bind the file to the package it is
            # shipped in, which is exactly what must not happen.
            if node.level:
                offenders.append(f"relative import at line {node.lineno}")
                continue
            names = [node.module or ""]
        else:
            continue

        for name in names:
            root = name.split(".")[0]
            if root and root not in stdlib:
                offenders.append(f"{name} at line {node.lineno}")

    assert not offenders, (
        "epics_tick.py must import only the standard library (architecture "
        "decision 7.6); it runs in Isaac Sim's embedded interpreter where "
        f"nothing else is installed. Offending imports: {offenders}"
    )


def test_tick_client_does_not_import_the_adapter():
    source = tick_source()
    assert b"isaac_epics" not in source
