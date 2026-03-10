import argparse
import json
import math
import os

from rigidbody2d_friction import FrictionConfig, FrictionScene, configure_prc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Improved 2D rigid body friction homework solution.")
    parser.add_argument("--headless", action="store_true", help="Run without an on-screen window.")
    parser.add_argument("--duration", type=float, default=6.0, help="Simulation duration in seconds.")
    parser.add_argument("--angle", type=float, default=13.0, help="Incline angle in degrees.")
    parser.add_argument("--mu", type=float, default=0.5, help="Static and kinetic friction coefficient.")
    parser.add_argument("--restitution", type=float, default=0.1, help="Coefficient of restitution.")
    parser.add_argument("--dt", type=float, default=1.0 / 120.0, help="Fixed time step.")
    parser.add_argument(
        "--solver-iterations",
        type=int,
        default=1,
        help="Velocity solver passes per frame. The extra version keeps the baseline default unless overridden.",
    )
    parser.add_argument("--screenshot", type=str, default=None, help="Optional PNG path for the final frame.")
    parser.add_argument("--report", type=str, default=None, help="Optional JSON path for the run report.")
    parser.add_argument(
        "--sweep",
        type=str,
        default=None,
        help="Optional JSON path. When set, a coefficient sweep is performed instead of a single run.",
    )
    return parser.parse_args()


def make_config(args: argparse.Namespace, mu: float | None = None) -> FrictionConfig:
    friction = args.mu if mu is None else mu
    return FrictionConfig(
        angle_deg=args.angle,
        dt=args.dt,
        restitution=args.restitution,
        mu_static=friction,
        mu_kinetic=friction,
        duration=args.duration,
        headless=args.headless,
        screenshot_path=args.screenshot,
        report_path=args.report,
        solver_iterations=args.solver_iterations,
        position_percent=1.0,
        position_slop=5.0e-4,
        enable_rest_snap=True,
        tangential_static_position_percent=1.0,
        tangential_sliding_position_percent=0.05,
        rest_speed_threshold=0.09,
        rest_omega_threshold=0.08,
        rest_contact_frames=6,
        disable_gravity_when_resting=True,
    )


def run_sweep(args: argparse.Namespace, output_path: str) -> None:
    sweep_values = [0.12, 0.18, 0.22, 0.24, 0.30, 0.50, 0.80]
    results = []
    for mu in sweep_values:
        config = make_config(args, mu=mu)
        config.headless = True
        config.screenshot_path = None
        config.report_path = None
        configure_prc(True)
        scene = FrictionScene(config)
        try:
            summary = scene.run_for_duration(config.duration)
        finally:
            scene.destroy()
        results.append(
            {
                "mu": mu,
                "final_tangent_velocity": summary["final_tangent_velocity"],
                "avg_tangent_accel_last_half": summary["avg_tangent_accel_last_half"],
                "total_tangent_displacement": summary["total_tangent_displacement"],
                "max_abs_tangent_velocity": summary["max_abs_tangent_velocity"],
                "contact_fraction": summary["contact_fraction"],
            }
        )

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as handle:
        json.dump(
            {
                "angle_deg": args.angle,
                "critical_mu": math.tan(math.radians(args.angle)),
                "results": results,
            },
            handle,
            indent=2,
        )


def main() -> None:
    args = parse_args()
    if args.sweep:
        run_sweep(args, args.sweep)
        return

    config = make_config(args)
    if not args.headless:
        config.headless = False
        config.screenshot_path = None
        configure_prc(False)
        scene = FrictionScene(config)
        scene.run()
        return

    configure_prc(config.headless)
    scene = FrictionScene(config)
    try:
        summary = scene.run_for_duration(config.duration)
    finally:
        scene.destroy()
    print(
        json.dumps(
            {
                "final_speed": summary["final_speed"],
                "final_tangent_velocity": summary["final_tangent_velocity"],
                "avg_tangent_accel_last_half": summary["avg_tangent_accel_last_half"],
                "contact_fraction": summary["contact_fraction"],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
