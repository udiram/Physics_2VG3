#!/usr/bin/env python3
"""2VG3 HW4 rigid body collision solver with optional Panda3D visualization."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

EPS = 1e-9


@dataclass
class Vec2:
    x: float
    z: float

    def __add__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x + other.x, self.z + other.z)

    def __sub__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x - other.x, self.z - other.z)

    def __mul__(self, scalar: float) -> "Vec2":
        return Vec2(self.x * scalar, self.z * scalar)

    __rmul__ = __mul__

    def __truediv__(self, scalar: float) -> "Vec2":
        return Vec2(self.x / scalar, self.z / scalar)

    def dot(self, other: "Vec2") -> float:
        return self.x * other.x + self.z * other.z

    def cross(self, other: "Vec2") -> float:
        # y-component of 3D cross in x-z plane.
        return self.x * other.z - self.z * other.x

    def mag(self) -> float:
        return math.hypot(self.x, self.z)

    def normalized(self) -> "Vec2":
        m = self.mag()
        if m < EPS:
            return Vec2(1.0, 0.0)
        return self / m

    def as_vec3_str(self, y: float = 25.0) -> str:
        return f"({self.x:.6f}, {y:.6f}, {self.z:.6f})"


class NodePath:
    """Minimal compatibility shim for class layout from lecture code."""

    def __init__(self, name: str = "node") -> None:
        self.name = name
        self._pos = Vec2(0.0, 0.0)

    def setPos(self, v3: Tuple[float, float, float]) -> None:
        self._pos = Vec2(v3[0], v3[2])

    def getPos2D(self) -> Vec2:
        return self._pos


@dataclass
class Particle(NodePath):
    vel: Vec2 = field(default_factory=lambda: Vec2(0.0, 0.0))
    inverseMass: float = 1.0
    radius: float = 1.0
    collisionRadius: float = 1.8

    def __init__(self, name: str = "p") -> None:
        NodePath.__init__(self, name)
        self.vel = Vec2(0.0, 0.0)
        self.inverseMass = 1.0
        self.radius = 1.0
        self.collisionRadius = 1.8


@dataclass
class RigidBody2D(Particle):
    theta: float = 0.0
    omega: float = 0.0
    inverseMomentOfInertia: float = 1.0

    def __init__(self, name: str = "p") -> None:
        Particle.__init__(self, name)
        self.theta = 0.0
        self.omega = 0.0
        # Match inertia to collision shape size so angular response is stable.
        self.inverseMomentOfInertia = self.inverseMass / (2.0 / 3.0 * self.collisionRadius**2)

    @property
    def mass(self) -> float:
        return 1.0 / self.inverseMass if self.inverseMass > EPS else float("inf")

    @property
    def inertia(self) -> float:
        return 1.0 / self.inverseMomentOfInertia if self.inverseMomentOfInertia > EPS else float("inf")

    @property
    def pos(self) -> Vec2:
        return self.getPos2D()

    @pos.setter
    def pos(self, v: Vec2) -> None:
        self.setPos((v.x, 25.0, v.z))

    def world_vertices(self) -> List[Vec2]:
        h = self.collisionRadius
        local = [Vec2(-h, -h), Vec2(h, -h), Vec2(h, h), Vec2(-h, h)]
        c = math.cos(self.theta)
        s = math.sin(self.theta)
        return [Vec2(c * p.x - s * p.z, s * p.x + c * p.z) + self.pos for p in local]

    def edge_normals(self) -> List[Vec2]:
        verts = self.world_vertices()
        normals: List[Vec2] = []
        for i in range(4):
            a = verts[i]
            b = verts[(i + 1) % 4]
            edge = b - a
            normals.append(Vec2(-edge.z, edge.x).normalized())
        return normals


@dataclass
class CollisionEvent:
    time: float
    normal: Vec2
    contact_points: List[Vec2]
    avg_point: Vec2


@dataclass
class StateSummary:
    lin_momentum: Vec2
    ang_momentum: float
    trans_energy: float
    rot_energy: float

    @property
    def total_energy(self) -> float:
        return self.trans_energy + self.rot_energy


def project(verts: Iterable[Vec2], axis: Vec2) -> Tuple[float, float]:
    vals = [v.dot(axis) for v in verts]
    return min(vals), max(vals)


def sat_collision(a: RigidBody2D, b: RigidBody2D) -> Tuple[bool, Vec2, float]:
    verts_a = a.world_vertices()
    verts_b = b.world_vertices()

    min_overlap = float("inf")
    best_axis = Vec2(1.0, 0.0)

    for axis in a.edge_normals()[:2] + b.edge_normals()[:2]:
        axis = axis.normalized()
        min_a, max_a = project(verts_a, axis)
        min_b, max_b = project(verts_b, axis)
        overlap = min(max_a, max_b) - max(min_a, min_b)
        if overlap < 0:
            return False, Vec2(0.0, 0.0), 0.0
        if overlap < min_overlap:
            min_overlap = overlap
            best_axis = axis

    if (b.pos - a.pos).dot(best_axis) < 0:
        best_axis = best_axis * -1.0

    return True, best_axis, min_overlap


def line_intersection(p1: Vec2, p2: Vec2, q1: Vec2, q2: Vec2) -> Optional[Vec2]:
    r = p2 - p1
    s = q2 - q1
    den = r.cross(s)
    if abs(den) < EPS:
        return None
    qp = q1 - p1
    t = qp.cross(s) / den
    u = qp.cross(r) / den
    if -EPS <= t <= 1 + EPS and -EPS <= u <= 1 + EPS:
        return p1 + r * t
    return None


def point_in_poly(pt: Vec2, poly: List[Vec2]) -> bool:
    sign = None
    n = len(poly)
    for i in range(n):
        a = poly[i]
        b = poly[(i + 1) % n]
        edge = b - a
        c = edge.cross(pt - a)
        if abs(c) < 1e-7:
            continue
        curr = c > 0
        if sign is None:
            sign = curr
        elif sign != curr:
            return False
    return True


def dedupe(points: List[Vec2], tol: float = 1e-6) -> List[Vec2]:
    out: List[Vec2] = []
    for p in points:
        if all((p - q).mag() > tol for q in out):
            out.append(p)
    return out


def intersection_contacts(a: RigidBody2D, b: RigidBody2D) -> List[Vec2]:
    va = a.world_vertices()
    vb = b.world_vertices()
    points: List[Vec2] = []

    for p in va:
        if point_in_poly(p, vb):
            points.append(p)
    for p in vb:
        if point_in_poly(p, va):
            points.append(p)

    for i in range(4):
        a1, a2 = va[i], va[(i + 1) % 4]
        for j in range(4):
            b1, b2 = vb[j], vb[(j + 1) % 4]
            inter = line_intersection(a1, a2, b1, b2)
            if inter is not None:
                points.append(inter)

    points = dedupe(points)
    if len(points) <= 2:
        return points

    # Keep two extreme contacts for face-face case.
    max_pair = (points[0], points[1])
    max_d = -1.0
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            d = (points[i] - points[j]).mag()
            if d > max_d:
                max_d = d
                max_pair = (points[i], points[j])
    return [max_pair[0], max_pair[1]]


def velocity_at_point(body: RigidBody2D, r: Vec2) -> Vec2:
    return body.vel + Vec2(-body.omega * r.z, body.omega * r.x)


def point_in_separation(a: RigidBody2D, b: RigidBody2D, normal: Vec2) -> float:
    min_a, max_a = project(a.world_vertices(), normal)
    min_b, max_b = project(b.world_vertices(), normal)
    return min(max_a, max_b) - max(min_a, min_b)


def resolve_collision(a: RigidBody2D, b: RigidBody2D, normal: Vec2, point: Vec2, restitution: float) -> None:
    ra = point - a.pos
    rb = point - b.pos

    rv = velocity_at_point(b, rb) - velocity_at_point(a, ra)
    vel_along_normal = rv.dot(normal)
    if vel_along_normal > 0:
        return

    ra_cn = ra.cross(normal)
    rb_cn = rb.cross(normal)
    denom = (
        a.inverseMass
        + b.inverseMass
        + ra_cn * ra_cn * a.inverseMomentOfInertia
        + rb_cn * rb_cn * b.inverseMomentOfInertia
    )
    if abs(denom) < EPS:
        return

    j = -(1.0 + restitution) * vel_along_normal / denom
    impulse = normal * j

    a.vel = a.vel - impulse * a.inverseMass
    b.vel = b.vel + impulse * b.inverseMass
    a.omega -= a.inverseMomentOfInertia * ra.cross(impulse)
    b.omega += b.inverseMomentOfInertia * rb.cross(impulse)

    separate_bodies(a, b, normal, point_in_separation(a, b, normal))


def separate_bodies(a: RigidBody2D, b: RigidBody2D, normal: Vec2, depth: float) -> None:
    """Positional correction only; no velocity impulse."""
    if depth <= 1e-4:
        return
    correction = normal * (0.5 * (depth - 1e-4))
    a.pos = a.pos - correction * a.inverseMass
    b.pos = b.pos + correction * b.inverseMass


def total_state(body0: RigidBody2D, body1: RigidBody2D) -> StateSummary:
    m0, m1 = body0.mass, body1.mass

    p = body0.vel * m0 + body1.vel * m1
    l_total = (
        m0 * body0.pos.cross(body0.vel)
        + m1 * body1.pos.cross(body1.vel)
        + body0.inertia * body0.omega
        + body1.inertia * body1.omega
    )

    k_t = 0.5 * m0 * body0.vel.dot(body0.vel) + 0.5 * m1 * body1.vel.dot(body1.vel)
    k_r = 0.5 * body0.inertia * body0.omega * body0.omega + 0.5 * body1.inertia * body1.omega * body1.omega
    return StateSummary(lin_momentum=p, ang_momentum=l_total, trans_energy=k_t, rot_energy=k_r)


def scenario_setup(index: int) -> Tuple[RigidBody2D, RigidBody2D]:
    p0 = RigidBody2D("body0")
    p1 = RigidBody2D("body1")

    p0.pos = Vec2(-6.0, -0.5)
    p1.pos = Vec2(11.0, 0.5)
    p0.vel = Vec2(2.0, 0.0)
    p1.vel = Vec2(-4.0, 0.0)

    if index == 1:
        p0.theta = math.radians(30.0)
    elif index == 2:
        p0.theta = math.radians(60.0)
        p1.theta = math.radians(30.0)
    elif index == 3:
        p0.omega = -5.0
        p1.vel = Vec2(0.0, 0.0)
        p1.pos = Vec2(-1.9, 1.9)
    elif index == 4:
        p0.omega = 3.0
        p1.theta = math.radians(-10.0)
    elif index != 0:
        raise ValueError(f"Unknown scenario {index}")

    return p0, p1


def run_simulation(
    scenario: int,
    restitution: float,
    t_end: float = 8.0,
    dt: float = 0.0005,
) -> Tuple[List[CollisionEvent], Optional[Tuple[StateSummary, StateSummary]]]:
    a, b = scenario_setup(scenario)

    events: List[CollisionEvent] = []
    in_contact = False
    first_snap: Optional[Tuple[StateSummary, StateSummary]] = None
    t = 0.0

    while t < t_end:
        a.pos = a.pos + a.vel * dt
        b.pos = b.pos + b.vel * dt
        a.theta += a.omega * dt
        b.theta += b.omega * dt

        hit, normal, _ = sat_collision(a, b)
        if hit:
            contacts = intersection_contacts(a, b)
            if contacts:
                avg = Vec2(sum(p.x for p in contacts) / len(contacts), sum(p.z for p in contacts) / len(contacts))
                if not in_contact:
                    events.append(CollisionEvent(t, normal, contacts, avg))
                    if first_snap is None:
                        before = total_state(a, b)
                        resolve_collision(a, b, normal, avg, restitution)
                        after = total_state(a, b)
                        first_snap = (before, after)
                    else:
                        resolve_collision(a, b, normal, avg, restitution)
                else:
                    # Persisting overlap: apply position-only correction.
                    separate_bodies(a, b, normal, point_in_separation(a, b, normal))
                in_contact = True
        else:
            in_contact = False

        t += dt

    return events, first_snap


def run_all_text_report() -> None:
    for e in (1.0, 0.0):
        print(f"=== e={int(e)} ===")
        for scenario in range(5):
            events, snap = run_simulation(scenario, e)
            if not events:
                print(f"Scenario {scenario}: no collisions")
                continue
            first = events[0]
            print(
                f"Scenario {scenario}: first collision t={first.time:.6f}, "
                f"avg={first.avg_point.as_vec3_str()}, normal={first.normal.as_vec3_str(0.0)}"
            )
            if snap is not None:
                before, after = snap
                print(
                    "  Before: "
                    f"P=({before.lin_momentum.x:.6f},{before.lin_momentum.z:.6f}) "
                    f"L={before.ang_momentum:.6f} K={before.total_energy:.6f}"
                )
                print(
                    "  After : "
                    f"P=({after.lin_momentum.x:.6f},{after.lin_momentum.z:.6f}) "
                    f"L={after.ang_momentum:.6f} K={after.total_energy:.6f}"
                )
        print()


def run_panda3d_viewer(
    scenario: int,
    restitution: float,
    seconds: float,
    offscreen: bool,
    screenshot_path: Optional[Path],
) -> None:
    try:
        from direct.showbase.ShowBase import ShowBase
        from panda3d.core import AmbientLight, CardMaker, DirectionalLight, Filename, LVector3, Quat, loadPrcFileData
    except ImportError as exc:
        raise SystemExit(
            "Panda3D is not installed. Install with: python3 -m pip install --user --break-system-packages panda3d"
        ) from exc

    if offscreen:
        loadPrcFileData("", "window-type offscreen")
    loadPrcFileData("", "win-size 1280 720")

    class Viewer(ShowBase):
        def __init__(self) -> None:
            super().__init__()
            self.disableMouse()

            self.p0, self.p1 = scenario_setup(scenario)
            self.restitution = restitution
            self.elapsed = 0.0
            self.fixed_dt = 1.0 / 240.0
            self.in_contact = False
            self.collision_point: Optional[Vec2] = None

            self.camera.setPos(2, -48, 22)
            self.camera.lookAt(2, 0, 0)

            amb = AmbientLight("amb")
            amb.setColor((0.75, 0.75, 0.78, 1.0))
            self.render.setLight(self.render.attachNewNode(amb))

            sun = DirectionalLight("sun")
            sun.setColor((0.65, 0.65, 0.60, 1.0))
            sun_np = self.render.attachNewNode(sun)
            sun_np.setHpr(35, -45, 0)
            self.render.setLight(sun_np)

            cm = CardMaker("floor")
            cm.setFrame(-24, 24, -24, 24)
            floor = self.render.attachNewNode(cm.generate())
            floor.setP(-90)
            floor.setPos(2, 24, -0.01)
            floor.setColor(0.88, 0.90, 0.93, 1.0)

            # Panda's models/box spans [0,1]^3 with pivot at a corner. We wrap it
            # so transform origin is at geometric center and visual size matches
            # collisionRadius (half extent 1.8 -> full width 3.6 in x/z).
            self.body0 = self.render.attachNewNode("body0")
            self.body1 = self.render.attachNewNode("body1")
            mesh0 = self.loader.loadModel("models/box")
            mesh1 = self.loader.loadModel("models/box")
            mesh0.reparentTo(self.body0)
            mesh1.reparentTo(self.body1)
            mesh0.setPos(-0.5, -0.5, -0.5)
            mesh1.setPos(-0.5, -0.5, -0.5)
            self.body0.setScale(3.6, 0.7, 3.6)
            self.body1.setScale(3.6, 0.7, 3.6)
            mesh0.setColor(0.2, 0.47, 0.83, 1.0)
            mesh1.setColor(0.86, 0.22, 0.22, 1.0)

            self.marker = self.loader.loadModel("models/smiley")
            self.marker.reparentTo(self.render)
            self.marker.setScale(0.15)
            self.marker.setColor(0.1, 0.1, 0.1, 1.0)
            self.marker.hide()

            self.update_visuals()
            self.taskMgr.add(self.step_task, "collision_step")

        def update_visuals(self) -> None:
            self.body0.setPos(self.p0.pos.x, 0, self.p0.pos.z)
            self.body1.setPos(self.p1.pos.x, 0, self.p1.pos.z)

            q0 = Quat()
            q0.setFromAxisAngleRad(self.p0.theta, LVector3(0, 1, 0))
            q1 = Quat()
            q1.setFromAxisAngleRad(self.p1.theta, LVector3(0, 1, 0))
            self.body0.setQuat(q0)
            self.body1.setQuat(q1)

            if self.collision_point is None:
                self.marker.hide()
            else:
                self.marker.show()
                self.marker.setPos(self.collision_point.x, 0.08, self.collision_point.z)

        def step_task(self, task):
            dt = min(globalClock.getDt(), 1 / 60)
            substeps = max(1, int(round(dt / self.fixed_dt)))

            for _ in range(substeps):
                dts = dt / substeps
                self.p0.pos = self.p0.pos + self.p0.vel * dts
                self.p1.pos = self.p1.pos + self.p1.vel * dts
                self.p0.theta += self.p0.omega * dts
                self.p1.theta += self.p1.omega * dts

                hit, normal, _ = sat_collision(self.p0, self.p1)
                if hit:
                    contacts = intersection_contacts(self.p0, self.p1)
                    if contacts:
                        avg = Vec2(
                            sum(c.x for c in contacts) / len(contacts),
                            sum(c.z for c in contacts) / len(contacts),
                        )
                    else:
                        avg = (self.p0.pos + self.p1.pos) * 0.5

                    if not self.in_contact:
                        resolve_collision(self.p0, self.p1, normal, avg, self.restitution)
                    else:
                        separate_bodies(self.p0, self.p1, normal, point_in_separation(self.p0, self.p1, normal))

                    self.collision_point = avg
                    self.in_contact = True
                else:
                    self.in_contact = False
                    self.collision_point = None

                self.elapsed += dts

            self.update_visuals()

            if self.elapsed >= seconds:
                if offscreen and screenshot_path is not None:
                    self.graphicsEngine.renderFrame()
                    self.win.saveScreenshot(Filename.fromOsSpecific(str(screenshot_path)))
                    print(f"Wrote screenshot: {screenshot_path}")
                self.userExit()
                return task.done

            return task.cont

    app = Viewer()
    mode = "offscreen" if offscreen else "window"
    print(f"Starting Panda3D {mode} viewer: scenario={scenario}, e={restitution}")
    app.run()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="HW4 rigidbody 2D collisions")
    p.add_argument("--scenario", type=int, default=0, help="scenario index 0..4")
    p.add_argument("--e", type=float, default=1.0, choices=[0.0, 1.0], help="coefficient of restitution")
    p.add_argument("--seconds", type=float, default=6.0, help="viewer run time")
    p.add_argument("--report", action="store_true", help="print text report for all scenarios instead of opening Panda3D")
    p.add_argument("--offscreen", action="store_true", help="render headless and auto-quit")
    p.add_argument("--screenshot", default="panda3d_sanity.png", help="screenshot output in offscreen mode")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    if args.report:
        run_all_text_report()
        return

    screenshot = Path(args.screenshot) if args.offscreen else None
    run_panda3d_viewer(
        scenario=args.scenario,
        restitution=args.e,
        seconds=args.seconds,
        offscreen=args.offscreen,
        screenshot_path=screenshot,
    )


if __name__ == "__main__":
    main()
