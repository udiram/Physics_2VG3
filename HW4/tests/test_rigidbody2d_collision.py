from __future__ import annotations

import math
from pathlib import Path
import sys

from panda3d.core import Vec3

# Ensure HW4 module can be imported when tests run from repo root.
HW4_DIR = Path(__file__).resolve().parents[1]
if str(HW4_DIR) not in sys.path:
    sys.path.insert(0, str(HW4_DIR))

from rigidbody2d_collision import CollisionEngine2D, run_all_cases, run_case


def vec_norm(v: list[float]) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def vec_delta(a: list[float], b: list[float]) -> list[float]:
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def test_all_scenarios_have_collisions_for_e_1_and_e_0() -> None:
    results = run_all_cases(dt=1.0 / 240.0, max_time=6.0, stop_after_first_collision=True)
    assert len(results) == 10
    for result in results:
        assert result["collision_count"] >= 1, (
            f"Scenario {result['scenario']} with e={result['restitution']} "
            "did not report any collision"
        )


def test_scenario0_e1_conserves_total_quantities_across_collision() -> None:
    result = run_case(
        scenario_id=0,
        restitution=1.0,
        dt=1.0 / 480.0,
        max_time=6.0,
        stop_after_first_collision=True,
    )
    assert result["collision_count"] >= 1

    first = result["collisions"][0]
    pre = first["pre_quantities"]
    post = first["post_quantities"]

    d_p = vec_norm(vec_delta(post["momentum"], pre["momentum"]))
    d_l = vec_norm(vec_delta(post["angular_momentum"], pre["angular_momentum"]))
    d_ke = abs(post["total_ke"] - pre["total_ke"])

    assert d_p < 1e-8
    assert d_l < 1e-8
    assert d_ke < 1e-6


def test_scenario0_e0_loses_energy_but_conserves_momentum_and_angular_momentum() -> None:
    result = run_case(
        scenario_id=0,
        restitution=0.0,
        dt=1.0 / 480.0,
        max_time=6.0,
        stop_after_first_collision=True,
    )
    assert result["collision_count"] >= 1

    first = result["collisions"][0]
    pre = first["pre_quantities"]
    post = first["post_quantities"]

    d_p = vec_norm(vec_delta(post["momentum"], pre["momentum"]))
    d_l = vec_norm(vec_delta(post["angular_momentum"], pre["angular_momentum"]))

    assert post["total_ke"] < pre["total_ke"] - 1e-6
    assert d_p < 1e-8
    assert d_l < 1e-8


def test_head_on_equal_mass_velocity_exchange_at_e1() -> None:
    engine = CollisionEngine2D(scenario_id=0, restitution=1.0)
    b0, b1 = engine.bodies

    b0.pos = Vec3(-3.0, 25.0, 0.0)
    b1.pos = Vec3(3.0, 25.0, 0.0)
    b0.vel = Vec3(1.0, 0.0, 0.0)
    b1.vel = Vec3(-2.0, 0.0, 0.0)
    b0.theta = 0.0
    b1.theta = 0.0
    b0.omega = 0.0
    b1.omega = 0.0

    engine.run(dt=1.0 / 600.0, max_time=6.0, stop_after_first_collision=True)
    assert len(engine.events) >= 1

    assert abs(b0.vel.x - (-2.0)) < 1e-4
    assert abs(b1.vel.x - 1.0) < 1e-4
    assert abs(b0.omega) < 1e-5
    assert abs(b1.omega) < 1e-5


def test_face_face_overlap_produces_multiple_contacts_for_averaging() -> None:
    engine = CollisionEngine2D(scenario_id=0, restitution=1.0)
    b0, b1 = engine.bodies

    b0.pos = Vec3(0.0, 25.0, 0.0)
    b1.pos = Vec3(1.9, 25.0, 0.0)
    b0.theta = 0.0
    b1.theta = 0.0

    manifold = engine.collide_square_square(b0, b1)
    assert manifold is not None
    assert len(manifold.contacts) >= 2

    xs = [c.point.x for c in manifold.contacts]
    zs = [c.point.z for c in manifold.contacts]
    avg_x = sum(xs) / len(xs)
    avg_z = sum(zs) / len(zs)

    assert abs(manifold.averaged_point.x - avg_x) < 1e-8
    assert abs(manifold.averaged_point.z - avg_z) < 1e-8
