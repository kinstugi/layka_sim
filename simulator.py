#!/usr/bin/env python3
"""Default entry point: runs the NEW ``layka`` simulator.

The new simulator is a GTK application built entirely from the clean
``layka`` package (search + LJ swarm aggregation + IR-sensor obstacle
avoidance + world-boundary containment). It reuses the generic legacy GUI
chrome (buttons/window/cairo painter) from ``legacy_code/gui``, which is why
this launcher adds ``legacy_code`` to ``sys.path`` before importing
:mod:`layka.sim`.

Run from the repository root:  ``uv run python simulator.py``
(alternatively: ``uv run python -m layka.sim``).

The OLD implementation still lives in ``legacy_code/`` and can be launched
with ``uv run python legacy_code/simulator.py``.
"""

import os
import sys

# Make the shared GUI chrome (legacy_code/gui) importable.
_here = os.path.dirname(os.path.abspath(__file__))
_legacy = os.path.join(_here, "legacy_code")
if _legacy not in sys.path:
    sys.path.insert(0, _legacy)

from layka.sim import LaykaSimController  # noqa: E402

if __name__ == "__main__":
    LaykaSimController()
