"""M2.12 tests: experiment runner CLI (json/csv, reproducibility)."""

from __future__ import annotations

import csv
import json
import subprocess
import sys
from pathlib import Path

import pytest

from experiments.aggregate import (
    build_parser,
    main,
    result_to_dict,
)
from layka.experiments import run_aggregation_experiment

REPO_ROOT = Path(__file__).resolve().parents[1]


def _run(argv: list[str]):
    return main(argv)


class TestParser:
    def test_defaults(self):
        args = build_parser().parse_args([])
        assert args.robots == 10
        assert args.spacing == 0.40
        assert args.seed == 42
        assert args.steps == 2000
        assert args.out is None
        assert args.format is None

    def test_flag_mapping(self):
        args = build_parser().parse_args(
            ["--robots", "5", "--spacing", "0.5", "--seed", "7", "--steps", "50",
             "--out", "x.json", "--format", "json"]
        )
        assert args.robots == 5
        assert args.spacing == 0.5
        assert args.seed == 7
        assert args.steps == 50
        assert args.out == "x.json"
        assert args.format == "json"


class TestResultToDict:
    def test_serializes_all_fields(self):
        result = run_aggregation_experiment(swarm_size=3, max_steps=20, random_seed=42)
        data = result_to_dict(result)
        for key in (
            "swarm_size",
            "desired_spacing",
            "random_seed",
            "num_steps",
            "converged",
            "centroid",
            "cluster_size",
            "cluster_fraction",
            "mean_pairwise_distance",
            "final_positions",
        ):
            assert key in data
        assert data["swarm_size"] == 3
        assert data["random_seed"] == 42
        assert data["centroid"] == [pytest.approx(result.centroid.x), pytest.approx(result.centroid.y)]
        assert isinstance(data["final_positions"], list)
        assert len(data["final_positions"]) == 3
        json.dumps(data)  # must be JSON-serializable


class TestJsonSaveAndReproducibility:
    def test_saves_json_and_reproduces_byte_identically(self, tmp_path):
        first = tmp_path / "a.json"
        second = tmp_path / "b.json"
        args = ["--robots", "5", "--steps", "50", "--seed", "42", "--out", str(first)]
        assert _run(args) == 0
        assert _run(args[:-1] + [str(second)]) == 0
        assert first.exists() and second.exists()
        assert first.read_bytes() == second.read_bytes()  # reproducibility

        data = json.loads(first.read_text())
        assert data["swarm_size"] == 5
        assert data["random_seed"] == 42
        assert data["num_steps"] >= 0
        assert 0.0 <= data["cluster_fraction"] <= 1.0

    def test_different_seed_diverges(self, tmp_path):
        a = tmp_path / "a.json"
        b = tmp_path / "b.json"
        assert _run(["--robots", "5", "--steps", "50", "--seed", "42", "--out", str(a)]) == 0
        assert _run(["--robots", "5", "--steps", "50", "--seed", "43", "--out", str(b)]) == 0
        assert a.read_bytes() != b.read_bytes()


class TestCsvSave:
    def test_saves_csv_with_scalar_metrics(self, tmp_path):
        out = tmp_path / "result.csv"
        assert _run(["--robots", "4", "--steps", "40", "--seed", "42", "--out", str(out)]) == 0
        with open(out, newline="") as handle:
            rows = list(csv.DictReader(handle))
        assert len(rows) == 1
        row = rows[0]
        assert int(row["swarm_size"]) == 4
        assert int(row["random_seed"]) == 42
        assert "cluster_fraction" in row
        assert "mean_pairwise_distance" in row
        assert "centroid_x" in row and "centroid_y" in row


class TestModuleEntry:
    def test_python_m_entry_works(self, tmp_path):
        out = tmp_path / "module.json"
        proc = subprocess.run(
            [sys.executable, "-m", "experiments.aggregate",
             "--robots", "3", "--steps", "20", "--seed", "1", "--out", str(out)],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
        )
        assert proc.returncode == 0, proc.stderr
        assert out.exists()
        assert json.loads(out.read_text())["swarm_size"] == 3


class TestValidation:
    @pytest.mark.parametrize("args", [
        ["--robots", "0", "--steps", "10"],
        ["--robots", "-1", "--steps", "10"],
        ["--robots", "5", "--steps", "0"],
        ["--robots", "5", "--spacing", "-0.1"],
    ])
    def test_invalid_arguments_return_error(self, args):
        assert _run(args) != 0

    def test_python_m_entry_rejects_invalid(self):
        proc = subprocess.run(
            [sys.executable, "-m", "experiments.aggregate", "--robots", "0", "--steps", "10"],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
        )
        assert proc.returncode != 0
