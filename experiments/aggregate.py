"""Command-line runner for the multi-robot LJ aggregation experiment (M2.12).

Runs :func:`layka.experiments.run_aggregation_experiment` from the terminal,
prints a human-readable summary, and optionally saves the full result to a
JSON or CSV file.

Reproducibility: the underlying experiment is fully deterministic (M2.8), so
invoking this runner with the same parameters and ``--seed`` always produces
identical saved results.

Examples::

    python -m experiments.aggregate --robots 10 --spacing 0.40 --seed 42 --steps 5000
    python -m experiments.aggregate --robots 5 --seed 7 --out result.json
    python -m experiments.aggregate --robots 5 --seed 7 --out result.csv
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from typing import TYPE_CHECKING

from layka.experiments import run_aggregation_experiment

if TYPE_CHECKING:
    from layka.experiments import AggregationResult


def build_parser() -> argparse.ArgumentParser:
    """Build the ``aggregate`` command-line parser."""
    parser = argparse.ArgumentParser(
        prog="experiments.aggregate",
        description="Run the deterministic multi-robot LJ aggregation experiment.",
    )
    parser.add_argument(
        "--robots", type=int, default=10, help="Number of robots (default 10)."
    )
    parser.add_argument(
        "--spacing",
        type=float,
        default=0.40,
        help="Desired robot-to-robot equilibrium spacing in meters (default 0.40).",
    )
    parser.add_argument(
        "--seed", type=int, default=42, help="Random seed for reproducibility."
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
        help="Path to write the result to (JSON or CSV; extension or --format "
        "selects the format). If omitted, a summary plus the full result are "
        "printed to stdout.",
    )
    parser.add_argument(
        "--format",
        choices=["json", "csv"],
        default=None,
        help="Output format for --out (default: inferred from the file "
        "extension, else json).",
    )
    return parser


def result_to_dict(result: AggregationResult) -> dict:
    """Convert an ``AggregationResult`` into a JSON-serializable plain dict."""
    return {
        "swarm_size": result.swarm_size,
        "desired_spacing": result.desired_spacing,
        "random_seed": result.random_seed,
        "num_steps": result.num_steps,
        "converged": result.converged,
        "centroid": [result.centroid.x, result.centroid.y],
        "cluster_size": result.cluster_size,
        "cluster_fraction": result.cluster_fraction,
        "mean_pairwise_distance": result.mean_pairwise_distance,
        "final_positions": [[p.x, p.y] for p in result.final_positions],
    }


def _scalar_row(result: AggregationResult) -> dict:
    """The scalar (non-position) metrics as a flat dict for CSV output."""
    return {
        "swarm_size": result.swarm_size,
        "desired_spacing": result.desired_spacing,
        "random_seed": result.random_seed,
        "num_steps": result.num_steps,
        "converged": int(result.converged),
        "cluster_size": result.cluster_size,
        "cluster_fraction": result.cluster_fraction,
        "mean_pairwise_distance": result.mean_pairwise_distance,
        "centroid_x": result.centroid.x,
        "centroid_y": result.centroid.y,
    }


def _infer_format(path: str, explicit: str | None) -> str:
    if explicit is not None:
        return explicit
    if path.endswith(".csv"):
        return "csv"
    return "json"


def _save(path: str, fmt: str, result: AggregationResult) -> None:
    if fmt == "csv":
        with open(path, "w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(_scalar_row(result).keys()))
            writer.writeheader()
            writer.writerow(_scalar_row(result))
        return
    with open(path, "w") as handle:
        json.dump(result_to_dict(result), handle, indent=2)


def _summary(result: AggregationResult, robots: int, spacing: float, seed: int, steps: int) -> str:
    return (
        f"aggregation experiment: robots={robots} spacing={spacing} "
        f"seed={seed} steps={steps}\n"
        f"  converged={result.converged} after {result.num_steps} steps\n"
        f"  cluster_fraction={result.cluster_fraction:.3f} "
        f"(cluster_size={result.cluster_size}/{result.swarm_size})\n"
        f"  mean_pairwise_distance={result.mean_pairwise_distance:.3f} m"
    )


def main(argv: list[str] | None = None) -> int:
    """Run the aggregation experiment CLI; returns the process exit code."""
    args = build_parser().parse_args(argv)
    if args.robots <= 0:
        print(f"error: --robots must be >= 1, got {args.robots}", file=sys.stderr)
        return 2
    if args.steps <= 0:
        print(f"error: --steps must be >= 1, got {args.steps}", file=sys.stderr)
        return 2
    if not math.isfinite(args.spacing) or args.spacing <= 0:
        print(f"error: --spacing must be finite and > 0, got {args.spacing}", file=sys.stderr)
        return 2

    result = run_aggregation_experiment(
        swarm_size=args.robots,
        desired_spacing=args.spacing,
        random_seed=args.seed,
        max_steps=args.steps,
    )
    print(
        _summary(
            result,
            robots=args.robots,
            spacing=args.spacing,
            seed=args.seed,
            steps=args.steps,
        )
    )

    if args.out is not None:
        fmt = _infer_format(args.out, args.format)
        _save(args.out, fmt, result)
    else:
        print(json.dumps(result_to_dict(result), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
