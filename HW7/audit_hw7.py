import json
import math
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
CONSTRAINT_DIR = ROOT / "constraint"
OUTDIR = ROOT / "audit_output"
OUTDIR.mkdir(exist_ok=True)
PYTHON = sys.executable


def run_case(name: str, args: list[str], history: bool = False) -> dict:
    summary_path = OUTDIR / f"{name}.json"
    shot_path = OUTDIR / f"{name}.png"
    history_path = OUTDIR / f"{name}_history.json"
    cmd = [
        PYTHON,
        "rigidbody2d_cc.py",
        "--offscreen",
        "--summary-json",
        str(summary_path),
        "--screenshot",
        str(shot_path),
        *args,
    ]
    if history:
        cmd.extend(["--history-json", str(history_path), "--history-stride", "1"])
    proc = subprocess.run(cmd, cwd=CONSTRAINT_DIR, text=True, capture_output=True, check=True)
    result = {
        "summary": json.loads(summary_path.read_text()),
        "stdout": proc.stdout,
        "stderr": proc.stderr,
        "screenshot": str(shot_path),
    }
    if history:
        result["history"] = json.loads(history_path.read_text())
    return result


def body(source: dict, name: str) -> dict:
    for item in source["bodies"]:
        if item["name"] == name:
            return item
    raise KeyError(name)


def body_speed(item: dict) -> float:
    return math.hypot(item["vx"], item["vz"])


def kinetic_proxy(item: dict) -> float:
    return item["vx"] ** 2 + item["vz"] ** 2


def sphere_names(summary: dict) -> list[str]:
    names = [item["name"] for item in summary["bodies"] if item["name"].startswith("sphere")]
    return sorted(names, key=lambda name: int(name.replace("sphere", "")))


def stack_names(summary: dict) -> list[str]:
    names = [item["name"] for item in summary["bodies"] if item["name"].startswith("stack")]
    return sorted(names, key=lambda name: int(name.replace("stack", "")))


def sample_map(sample: dict) -> dict:
    return {item["name"]: item for item in sample["bodies"]}


def first_far_end_apex(trace: dict) -> dict | None:
    summary = trace["summary"]
    names = sphere_names(summary)
    if not names:
        return None
    rest_x = summary["cradle_rest_x"]
    rest_lookup = dict(zip(names, rest_x))
    far_name = names[-1]
    far_rest_x = rest_lookup[far_name]
    samples = trace["samples"]

    for prev_sample, cur_sample, next_sample in zip(samples, samples[1:], samples[2:]):
        prev_map = sample_map(prev_sample)
        cur_map = sample_map(cur_sample)
        next_map = sample_map(next_sample)
        far_x = cur_map[far_name]["x"]
        if cur_sample["time"] < 1.4 or far_x < far_rest_x + 0.05:
            continue
        if far_x < prev_map[far_name]["x"] or far_x < next_map[far_name]["x"]:
            continue

        other_names = names[:-1]
        residuals = {name: cur_map[name]["x"] - rest_lookup[name] for name in names}
        other_speeds = {name: body_speed(cur_map[name]) for name in other_names}
        max_other_residual = max(abs(residuals[name]) for name in other_names)
        total_ke = sum(kinetic_proxy(cur_map[name]) for name in names)
        far_ke = kinetic_proxy(cur_map[far_name])
        return {
            "time": cur_sample["time"],
            "far_name": far_name,
            "far_x": far_x,
            "far_displacement": far_x - far_rest_x,
            "far_to_non_end_displacement_ratio": ((far_x - far_rest_x) / max_other_residual) if max_other_residual else None,
            "far_speed": body_speed(cur_map[far_name]),
            "far_ke_ratio": (far_ke / total_ke) if total_ke else None,
            "residual_x": residuals,
            "non_end_max_abs_residual_x": max_other_residual,
            "non_end_max_speed": max(other_speeds.values()) if other_speeds else 0.0,
            "non_end_speeds": other_speeds,
        }
    return None


def baseline_checks() -> dict:
    cases = {"0": ("0.8", 0.8), "1": ("1.5", 1.5)}
    results = {}
    for scenario, (_, duration) in cases.items():
        run = run_case(f"scenario_{scenario}_t{str(duration).replace('.', 'p')}", ["--scenario", scenario, "--duration", str(duration)])
        summary = run["summary"]
        results[scenario] = {
            "time": summary["time"],
            "bodies": {
                item["name"]: {"x": item["x"], "z": item["z"], "vx": item["vx"], "vz": item["vz"]} for item in summary["bodies"]
            },
            "screenshot": run["screenshot"],
        }
    return results


def pendulum_checks() -> dict:
    theory = 2.0 * math.pi * math.sqrt(2.0 / 10.0)
    values = []
    cases = {}
    for vx in (-0.5, -1.0, -2.0, -4.0):
        run = run_case(
            f"pendulum_vx_{str(vx).replace('-', 'm').replace('.', 'p')}",
            ["--scenario", "2", "--duration", "8", "--pendulum-vx", str(vx)],
        )
        summary = run["summary"]
        measured = summary["pendulum_period_measured"]
        values.append(measured)
        cases[str(vx)] = {
            "measured": measured,
            "theory": theory,
            "abs_error": abs(measured - theory),
            "rel_error": abs(measured - theory) / theory,
            "crossings": summary["pendulum_crossings"],
            "screenshot": run["screenshot"],
        }
    return {
        "theory": theory,
        "period_increases_with_speed": all(values[i] < values[i + 1] for i in range(len(values) - 1)),
        "cases": cases,
    }


def cradle_case(name: str, args: list[str]) -> dict:
    trace_run = run_case(name, args, history=True)
    summary = trace_run["summary"]
    apex = first_far_end_apex(trace_run["history"])
    apex_shot = trace_run["screenshot"]
    if apex is not None:
        snapshot = run_case(f"{name}_apex", [*args, "--duration", f"{apex['time']:.6f}"])
        apex_shot = snapshot["screenshot"]
    return {
        "bias_mode": summary["bias_mode"],
        "cradle_gap": summary["cradle_gap"],
        "rest_x": summary["cradle_rest_x"],
        "first_far_end_apex": apex,
        "final_movers": summary["cradle_moving"],
        "final_speeds": {
            item["name"]: body_speed(item) for item in summary["bodies"] if item["name"].startswith("sphere")
        },
        "screenshot": apex_shot,
    }


def cradle_checks() -> dict:
    results = {}
    for bias in ("catto", "zero", "adaptive"):
        results[bias] = cradle_case(f"cradle_{bias}", ["--scenario", "3", "--duration", "6", "--bias-mode", bias])
    results["adaptive_ns5"] = cradle_case(
        "cradle_adaptive_ns5",
        ["--scenario", "3", "--duration", "6", "--bias-mode", "adaptive", "--nsphere", "5"],
    )
    return results


def stack_metrics(trace: dict) -> dict:
    summary = trace["summary"]
    names = stack_names(summary)
    samples = trace["samples"]
    min_gap = math.inf
    max_abs_vz_after_settle = 0.0
    max_abs_x_after_settle = 0.0

    for sample in samples:
        sample_bodies = sample_map(sample)
        z_values = [sample_bodies[name]["z"] for name in names]
        gaps = [z_values[i + 1] - z_values[i] for i in range(len(z_values) - 1)]
        if gaps:
            min_gap = min(min_gap, min(gaps))
        if sample["time"] >= 2.0:
            max_abs_vz_after_settle = max(max_abs_vz_after_settle, max(abs(sample_bodies[name]["vz"]) for name in names))
            max_abs_x_after_settle = max(max_abs_x_after_settle, max(abs(sample_bodies[name]["x"]) for name in names))

    final_stacks = [body(summary, name) for name in names]
    final_z = [item["z"] for item in final_stacks]
    final_gaps = [final_z[i + 1] - final_z[i] for i in range(len(final_z) - 1)]
    return {
        "final_z": final_z,
        "final_gaps": final_gaps,
        "min_gap_over_time": min_gap,
        "max_abs_vz_after_2s": max_abs_vz_after_settle,
        "max_abs_x_after_2s": max_abs_x_after_settle,
    }


def stack_checks() -> dict:
    results = {}
    for bias in ("catto", "zero", "adaptive"):
        run = run_case(f"stack_{bias}", ["--scenario", "4", "--duration", "12", "--bias-mode", bias], history=True)
        results[bias] = {**stack_metrics(run["history"]), "screenshot": run["screenshot"]}
    return results


def main() -> None:
    report = {
        "baseline": baseline_checks(),
        "pendulum": pendulum_checks(),
        "cradle": cradle_checks(),
        "stack": stack_checks(),
    }
    report_path = OUTDIR / "audit_report.json"
    report_path.write_text(json.dumps(report, indent=2))
    print(report_path)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
