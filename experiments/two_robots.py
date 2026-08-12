"""Command-line runner for the two-robot LJ aggregation experiment (M2.12).

Runs :func:`layka.experiments.run_two_robot_experiment` from the terminal and
optionally saves the result to JSON or CSV.

Reproducibility: the experiment is fully deterministic (M2.7), so the same
parameters always produce the same result.

Example::

    python -m experiments.two_robots --separation 0.60 --spacing 0.40 --steps 2000
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from typing import TYPE_CHECKING

from layka.experiments import run_two_robot_experiment

if TYPE_CHECKING:
    from layka.experiments import TwoRobotResult


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="experiments.two_robots",
        description="Run the deterministic two-robot LJ aggregation experiment.",
    )
    parser.add_argument(
        "--separation",
        type=float,
        default=0.60,
        help="Initial separation of robot B from robot A at (0, 0), in meters "
        "(default 0.60; > 0.40 is the attractive case, < 0.40 the repulsive one).",
    )
    parser.add_argument(
        "--spacing",
        type=float,
        default=0.40,
        help="Desired equilibrium spacing in meters (default 0.40).",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=2000,
        help="Maximum number of simulation steps to run (default 2000).",
    )
    parser.add_argument(
        "--out",
        type=str,
        default=None,
        help="Path to write the result to (JSON or CSV). If omitted, the "
        "result is printed as JSON to stdout.",
    )
    return parser


def result_to_dict(result: TwoRobotResult) -> dict:
    """Convert a ``TwoRobotResult`` into a JSON-serializable plain dict."""
    return {
        "initial_distance": result.initial_distance,
        "final_distance": result.final_distance,
        "equilibrium_target": result.equilibrium_target,
        "num_steps": result.num_steps,
        "min_distance": result.min_distance,
        "converged": result.converged,
    }


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.separation <= 0:
        print(f"error: --separation must be > 0, got {args.separation}", file=sys.stderr)
        return 2
    if args.steps <= 0:
        print(f"error: --steps must be >= 1, got {args.steps}", file=sys.stderr)
        return 2

    result = run_two_robot_experiment(
        initial_separation=args.separation,
        desired_spacing=args.spacing,
        max_steps=args.steps,
    )
    print(
        f"two-robot experiment: separation={args.separation} spacing={args.spacing} "
        f"steps={args.steps}\n"
        f"  initial={result.initial_distance:.3f} m -> "
        f"final={result.final_distance:.3f} m "
        f"(equilibrium {result.equilibrium_target:.3f} m) after "
        f"{result.num_steps} steps; converged={result.converged}"
    )
    if args.out is not None:
        if args.out.endswith(".csv"):
            with open(args.out, "w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(result_to_dict(result).keys()))
                writer.writeheader()
                writer.writerow(result_to_dict(result))
        else:
            with open(args.out, "w") as handle:
                json.dump(result_to_dict(result), handle, indent=2)
    else:
        print(json.dumps(result_to_dict(result), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
