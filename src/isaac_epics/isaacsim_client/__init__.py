"""The Isaac Sim side of the link, shipped so it can be handed out.

This subpackage exists to put :mod:`epics_tick` inside the wheel. The file is
*not* imported by the adapter and must never be: it runs in Isaac Sim's
embedded interpreter, which has its own pinned dependency set, and it is
written to the stdlib-only constraint in architecture decision 7.6.

It lives here so that ``pip install isaac-epics`` is enough to obtain it --
``isaac-epics-tick --install <dir>`` writes out the copy that matches the
installed adapter. Before, the file sat outside the packaged tree and the only
way to get it was to clone the repository, which made a pip install of the
adapter unable to talk to a simulator by construction.
"""
