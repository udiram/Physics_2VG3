from __future__ import annotations

import argparse
import copy
import json

from rigidbody2d_seq import (
    DEFAULT_BIAS_BETA,
    DEFAULT_BIAS_SLOP,
    DEFAULT_DT,
    SimpleScene,
    configure_prc,
)


EXERCISE2_SCENARIOS = (0, 1)
DEFAULT_EXERCISE2_ITERATIONS = 30


class Exercise2Scene(SimpleScene):
    def __init__(self, args: argparse.Namespace):
        normalized_args = copy.copy(args)
        if normalized_args.scenario not in EXERCISE2_SCENARIOS:
            normalized_args.scenario = EXERCISE2_SCENARIOS[0]
        super().__init__(normalized_args)

    def change_scenario(self, delta: int) -> None:
        if self.scenario not in EXERCISE2_SCENARIOS:
            self.scenario = EXERCISE2_SCENARIOS[0]
        current_index = EXERCISE2_SCENARIOS.index(self.scenario)
        self.scenario = EXERCISE2_SCENARIOS[(current_index + delta) % len(EXERCISE2_SCENARIOS)]
        self.setup_scenario()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Dedicated Panda3D runner for Homework 6 Exercise 2.")
    parser.add_argument("--scenario", type=int, choices=EXERCISE2_SCENARIOS, default=0, help="Exercise 2 scenario id.")
    parser.add_argument(
        "--iterations",
        type=int,
        default=DEFAULT_EXERCISE2_ITERATIONS,
        help="Sequential impulse iterations. Default chosen from the Exercise 2 sweep.",
    )
    parser.add_argument("--dt", type=float, default=DEFAULT_DT, help="Fixed timestep.")
    parser.add_argument("--duration", type=float, default=4.0, help="Headless run duration in seconds.")
    parser.add_argument("--bias-beta", type=float, default=DEFAULT_BIAS_BETA, help="Velocity bias factor.")
    parser.add_argument("--bias-slop", type=float, default=DEFAULT_BIAS_SLOP, help="Bias slop threshold.")
    parser.add_argument(
        "--bias-on-elastic",
        action="store_true",
        help="Also apply velocity bias when restitution is near 1.0.",
    )
    parser.add_argument("--screenshot", type=str, default=None, help="Optional final PNG output path.")
    parser.add_argument("--frame-dir", type=str, default=None, help="Optional directory for per-frame PNG capture.")
    parser.add_argument("--capture-every", type=int, default=2, help="Capture every N simulation frames when dumping.")
    parser.add_argument("--headless", action="store_true", help="Run without opening a window.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    configure_prc(args.headless)
    scene = Exercise2Scene(args)
    if args.headless:
        try:
            summary = scene.run_for_duration(args.duration)
            if args.screenshot:
                scene.save_screenshot(args.screenshot)
            print(json.dumps(summary, indent=2))
        finally:
            scene.destroy()
        return
    scene.run()


if __name__ == "__main__":
    main()
