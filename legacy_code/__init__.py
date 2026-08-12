"""The legacy Sobot-Rimulator-style simulation codebase, preserved as-is.

Everything here (``gui/``, ``models/``, ``robot_control/``, ``views/``,
``utils/``, ``sim_exceptions/``, ``maps/`` and ``simulator.py``) is the OLD
implementation. It is no longer the default entry point: ``python
simulator.py`` at the repository root now runs the new ``layka`` simulator.
The legacy simulator can still be launched with ``uv run python
legacy_code/simulator.py``.

Note: this package exists so the directory is importable; the legacy modules
use absolute imports (``from models.pose import Pose``, ``import gui.frame``,
...), which resolve because ``legacy_code/simulator.py`` inserts this
directory at the front of ``sys.path``. The new ``layka`` code does NOT import
any of these modules; it only reuses the generic GUI chrome
(``gui.viewer``/``gui.frame``/``gui.painter``/``gui.color_palette``) via the
same path shim.
"""
