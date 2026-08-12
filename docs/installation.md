# Installation

This project is a GTK3 application written in Python. It has exactly one
runtime Python dependency — [PyGObject](https://pygobject.readthedocs.io/)
— plus the system GTK3 libraries that PyGObject binds to. Development only
adds `pytest`.

## Requirements

- Python >= 3.10 (CI and this repository are tested with 3.12).
- System packages for GTK3 + gobject-introspection. On Debian/Ubuntu:

  ```
  sudo apt install python3-venv python3-dev libgtk-3-dev libgirepository1.0-dev gir1.2-gtk-3.0 pkg-config
  ```

  `PyGObject` installs from source on Linux, so the `-dev` packages above are
  required to build it. If your OS ships a binary `pygobject` package (e.g.
  `python3-gi` on Debian/Ubuntu), you can skip the `-dev` packages and use
  that instead.
- A display server (X11/Wayland). The application is **not headless**: it
  exits at import time with `"GUI code not find the display."` when no
  display is available.

## Option 1 (recommended): uv

Install [uv](https://docs.astral.sh/uv/) first, then from the repository root:

```
uv sync
```

This creates a `.venv/`, installs the runtime dependency (`PyGObject`) and the
development group (`pytest`). The interpreter is selected from
`.python-version`.

Run the simulator (requires a display):

```
uv run python simulator.py
```

Run the tests:

```
uv run pytest
```

## Option 2: pip + venv

From the repository root:

```
python3 -m venv .venv
source .venv/bin/activate
pip install -e .          # installs PyGObject (needs the -dev packages above)
pip install pytest        # development dependency
```

Run the simulator (requires a display):

```
python simulator.py
```

Run the tests:

```
pytest
```

## Notes

- `python simulator.py` runs the **new `layka` simulator** (search + LJ swarm
  aggregation + IR-sensor obstacle avoidance). The generic GTK window/buttons
  are reused from `legacy_code/gui`, which is why the launcher and
  `layka/sim.py` add `legacy_code` to `sys.path`.
- The **legacy** Sobot-Rimulator code is preserved untouched under
  `legacy_code/` and can be launched with
  `uv run python legacy_code/simulator.py`.
- The new `layka` package imports only the Python standard library plus
  PyGObject (`import gi`) and Pydantic; it has no numpy dependency.
- `simulator.py` must be run from the repository root; the legacy modules
  (`legacy_code/gui/`, `legacy_code/models/`, ...) are imported by path and
  are not installed as a package.
- Headless limitation: `python simulator.py` fails without a display
  (`legacy_code/gui/viewer.py` checks `Gdk.Display.get_default()` and exits
  at import time). This is a known blocker documented in
  `docs/current_architecture.md`; the tests deliberately avoid importing
  `gi` so they run anywhere.
