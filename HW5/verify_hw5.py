import json
import os
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
OUTPUT = ROOT / "output"


def run_command(args: list[str]) -> None:
    result = subprocess.run(args, cwd=ROOT, capture_output=True, text=True)
    if result.returncode == 0:
        return
    if result.stdout:
        print(result.stdout)
    if result.stderr:
        print(result.stderr, file=sys.stderr)
    raise subprocess.CalledProcessError(result.returncode, args)


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def ensure_parent(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def verify_friction_baseline(summary: dict, sweep: dict) -> None:
    if summary["final_tangent_velocity"] > 0.2:
        raise AssertionError("Baseline friction default case is drifting too quickly.")

    by_mu = {round(entry["mu"], 2): entry for entry in sweep["results"]}
    if by_mu[0.12]["avg_tangent_accel_last_half"] < 1.0:
        raise AssertionError("Low-friction incline case should accelerate clearly.")
    if abs(by_mu[0.50]["final_tangent_velocity"]) > 0.1:
        raise AssertionError("High-friction baseline case should be close to resting.")


def verify_friction_extra(summary: dict, sweep: dict) -> None:
    if abs(summary["final_tangent_velocity"]) > 0.05:
        raise AssertionError("Improved friction case should settle almost completely.")
    if not summary.get("final_sleeping", False):
        raise AssertionError("Improved friction case should enter the resting sleep state.")
    if summary.get("final_gravity_enabled", True):
        raise AssertionError("Improved friction case should disable gravity once resting.")

    by_mu = {round(entry["mu"], 2): entry for entry in sweep["results"]}
    if by_mu[0.24]["final_tangent_velocity"] != 0.0:
        raise AssertionError("Improved solver should snap to rest just above the critical angle.")
    if by_mu[0.22]["avg_tangent_accel_last_half"] < 0.05:
        raise AssertionError("Improved solver should still slide below the critical angle.")


def verify_rolling(default_summary: dict, race: dict) -> None:
    if abs(default_summary["percent_error"]) > 0.5:
        raise AssertionError("Default rolling speed differs too much from the analytic value.")

    results = race["results"]
    ordered = [entry["scenario"] for entry in sorted(results, key=lambda item: item["measured_exit_speed"], reverse=True)]
    if ordered != ["slider", "sphere", "solid", "hollow"]:
        raise AssertionError(f"Unexpected race order: {ordered}")
    for entry in results:
        if abs(entry["percent_error"]) > 0.5:
            raise AssertionError(f"Race measurement for {entry['scenario']} is too inaccurate.")


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)

    baseline_report = OUTPUT / "friction_default.json"
    baseline_sweep = OUTPUT / "friction_sweep.json"
    baseline_png = OUTPUT / "friction_default.png"
    extra_report = OUTPUT / "friction_extra_default.json"
    extra_sweep = OUTPUT / "friction_extra_sweep.json"
    extra_png = OUTPUT / "friction_extra_default.png"
    rolling_report = OUTPUT / "rolling_default.json"
    rolling_race = OUTPUT / "rolling_race.json"
    rolling_png = OUTPUT / "rolling_default.png"

    for path in (
        baseline_report,
        baseline_sweep,
        baseline_png,
        extra_report,
        extra_sweep,
        extra_png,
        rolling_report,
        rolling_race,
        rolling_png,
    ):
        ensure_parent(path)

    run_command(
        [
            sys.executable,
            "rigidbody2d_friction.py",
            "--headless",
            "--duration",
            "3.0",
            "--report",
            str(baseline_report),
            "--screenshot",
            str(baseline_png),
        ]
    )
    run_command([sys.executable, "rigidbody2d_friction.py", "--headless", "--sweep", str(baseline_sweep)])

    run_command(
        [
            sys.executable,
            "rigidbody2d_friction_extra.py",
            "--headless",
            "--duration",
            "3.0",
            "--report",
            str(extra_report),
            "--screenshot",
            str(extra_png),
        ]
    )
    run_command([sys.executable, "rigidbody2d_friction_extra.py", "--headless", "--sweep", str(extra_sweep)])

    run_command(
        [
            sys.executable,
            "rigidbody2d_rolling.py",
            "--headless",
            "--report",
            str(rolling_report),
        ]
    )
    run_command(
        [
            sys.executable,
            "rigidbody2d_rolling.py",
            "--headless",
            "--duration",
            "1.0",
            "--screenshot",
            str(rolling_png),
        ]
    )
    run_command([sys.executable, "rigidbody2d_rolling.py", "--headless", "--race-report", str(rolling_race)])

    baseline_summary = load_json(baseline_report)
    baseline_sweep_summary = load_json(baseline_sweep)
    extra_summary = load_json(extra_report)
    extra_sweep_summary = load_json(extra_sweep)
    rolling_summary = load_json(rolling_report)
    rolling_race_summary = load_json(rolling_race)

    verify_friction_baseline(baseline_summary, baseline_sweep_summary)
    verify_friction_extra(extra_summary, extra_sweep_summary)
    verify_rolling(rolling_summary, rolling_race_summary)

    verification = {
        "status": "ok",
        "artifacts": {
            "friction_default_report": str(baseline_report.relative_to(ROOT)),
            "friction_default_image": str(baseline_png.relative_to(ROOT)),
            "friction_sweep_report": str(baseline_sweep.relative_to(ROOT)),
            "friction_extra_report": str(extra_report.relative_to(ROOT)),
            "friction_extra_image": str(extra_png.relative_to(ROOT)),
            "friction_extra_sweep_report": str(extra_sweep.relative_to(ROOT)),
            "rolling_default_report": str(rolling_report.relative_to(ROOT)),
            "rolling_default_image": str(rolling_png.relative_to(ROOT)),
            "rolling_race_report": str(rolling_race.relative_to(ROOT)),
        },
        "checks": {
            "baseline_rest_case_final_tangent_velocity": baseline_summary["final_tangent_velocity"],
            "extra_rest_case_final_tangent_velocity": extra_summary["final_tangent_velocity"],
            "extra_rest_case_sleeping": extra_summary["final_sleeping"],
            "rolling_default_percent_error": rolling_summary["percent_error"],
            "race_order": [
                entry["scenario"]
                for entry in sorted(
                    rolling_race_summary["results"], key=lambda item: item["measured_exit_speed"], reverse=True
                )
            ],
        },
    }

    verification_path = OUTPUT / "verification_summary.json"
    with verification_path.open("w", encoding="utf-8") as handle:
        json.dump(verification, handle, indent=2)

    print(json.dumps(verification, indent=2))


if __name__ == "__main__":
    main()
