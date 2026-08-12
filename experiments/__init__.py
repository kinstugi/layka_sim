"""Clean, headless experiment runners (M2.12).

A small set of command-line programs for running the deterministic ``layka``
experiments WITHOUT the GUI -- the same experiments exposed programmatically
by :mod:`layka.experiments`. Running an experiment with the same parameters
and random seed always produces the same result, and results can be saved to
JSON or CSV.

Available runners:

- ``python -m experiments.aggregate``  -- multi-robot LJ aggregation.
- ``python -m experiments.two_robots`` -- the two-robot aggregation scenario.

These runners are pure Python (no GTK); they import only the ``layka`` package
and the Python standard library.
"""
