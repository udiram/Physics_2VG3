from __future__ import annotations


import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

from direct.showbase.ShowBase import ShowBase
from direct.showbase.ShowBaseGlobal import globalClock
from direct.task import Task
from panda3d.core import AmbientLight, DirectionalLight, NodePath, Vec3


scenario = 1


EPS = 1e-9


def deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0


def vec3_to_list(v: Vec3) -> list[float]:
    return [float(v.x), float(v.y), float(v.z)]


def vec3_to_str(v: Vec3) -> str:
    return f"({v.x}, {v.y}, {v.z})"


def vec2_add(a: tuple[float, float], b: tuple[float, float]) -> tuple[float, float]:
    return (a[0] + b[0], a[1] + b[1])


def vec2_sub(a: tuple[float, float], b: tuple[float, float]) -> tuple[float, float]:
    return (a[0] - b[0], a[1] - b[1])


def vec2_dot(a: tuple[float, float], b: tuple[float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1]


def vec2_cross(a: tuple[float, float], b: tuple[float, float]) -> float:
    return a[0] * b[1] - a[1] * b[0]


def vec2_mul(a: tuple[float, float], s: float) -> tuple[float, float]:
    return (a[0] * s, a[1] * s)


def vec2_len(a: tuple[float, float]) -> float:
    return math.hypot(a[0], a[1])


def vec2_norm(a: tuple[float, float]) -> tuple[float, float]:
    n = vec2_len(a)
    if n < EPS:
        return (1.0, 0.0)
    return (a[0] / n, a[1] / n)


def vec2_avg(points: list[tuple[float, float]]) -> tuple[float, float]:
    if not points:
        return (0.0, 0.0)
    sx = 0.0
    sz = 0.0
    for px, pz in points:
        sx += px
        sz += pz
    inv_n = 1.0 / len(points)
    return (sx * inv_n, sz * inv_n)


def rotate_xz(local_x: float, local_z: float, theta: float) -> tuple[float, float]:
    c = math.cos(theta)
    s = math.sin(theta)
    world_x = local_x * c + local_z * s
    world_z = -local_x * s + local_z * c
    return (world_x, world_z)


def point_on_segment_distance(pt: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
    ab = vec2_sub(b, a)
    ap = vec2_sub(pt, a)
    denom = vec2_dot(ab, ab)
    if denom < EPS:
        return vec2_len(vec2_sub(pt, a))
    t = max(0.0, min(1.0, vec2_dot(ap, ab) / denom))
    closest = vec2_add(a, vec2_mul(ab, t))
    return vec2_len(vec2_sub(pt, closest))


def segment_intersection(
    p1: tuple[float, float],
    p2: tuple[float, float],
    q1: tuple[float, float],
    q2: tuple[float, float],
    tol: float = 1e-9,
) -> Optional[tuple[float, float]]:
    r = vec2_sub(p2, p1)
    s = vec2_sub(q2, q1)
    denom = vec2_cross(r, s)
    qp = vec2_sub(q1, p1)

    if abs(denom) < tol:
        return None

    t = vec2_cross(qp, s) / denom
    u = vec2_cross(qp, r) / denom
    if -tol <= t <= 1.0 + tol and -tol <= u <= 1.0 + tol:
        return vec2_add(p1, vec2_mul(r, t))
    return None


def point_in_convex_polygon(pt: tuple[float, float], poly: list[tuple[float, float]], tol: float = 1e-8) -> bool:
    has_pos = False
    has_neg = False
    count = len(poly)
    for i in range(count):
        a = poly[i]
        b = poly[(i + 1) % count]
        edge = vec2_sub(b, a)
        rel = vec2_sub(pt, a)
        c = vec2_cross(edge, rel)
        if c > tol:
            has_pos = True
        elif c < -tol:
            has_neg = True
        if has_pos and has_neg:
            return False
    return True


def dedupe_points(points: list[tuple[float, float]], tol: float = 1e-6) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    tol2 = tol * tol
    for p in points:
        duplicate = False
        for q in out:
            dx = p[0] - q[0]
            dz = p[1] - q[1]
            if dx * dx + dz * dz <= tol2:
                duplicate = True
                break
        if not duplicate:
            out.append(p)
    return out


def reduce_contact_points(
    points: list[tuple[float, float]],
    normal: tuple[float, float],
    verts_i: list[tuple[float, float]],
    verts_j: list[tuple[float, float]],
    penetration: float,
) -> list[tuple[float, float]]:
    if len(points) <= 2:
        return points

    nx, nz = vec2_norm(normal)
    tangent = (-nz, nx)

    max_i = max(vec2_dot(v, (nx, nz)) for v in verts_i)
    min_j = min(vec2_dot(v, (nx, nz)) for v in verts_j)
    contact_plane = 0.5 * (max_i + min_j)

    plane_tol = max(1e-4, 2.0 * penetration + 1e-6)
    near_plane = [p for p in points if abs(vec2_dot(p, (nx, nz)) - contact_plane) <= plane_tol]
    candidates = near_plane if near_plane else points

    if len(candidates) <= 2:
        return candidates

    min_p = min(candidates, key=lambda p: vec2_dot(p, tangent))
    max_p = max(candidates, key=lambda p: vec2_dot(p, tangent))
    reduced = dedupe_points([min_p, max_p], tol=1e-7)

    if len(reduced) == 1:
        return reduced
    return reduced[:2]


@dataclass
class ContactDetail:
    point: Vec3
    classification: str
    corner_body: Optional[int]
    face_body: Optional[int]

    def to_dict(self) -> dict[str, Any]:
        return {
            "point": vec3_to_list(self.point),
            "classification": self.classification,
            "corner_body": self.corner_body,
            "face_body": self.face_body,
        }


@dataclass
class CollisionManifold:
    normal: Vec3
    penetration: float
    averaged_point: Vec3
    contacts: list[ContactDetail]


@dataclass
class SystemQuantities:
    momentum: Vec3
    angular_momentum: Vec3
    translational_ke: float
    rotational_ke: float

    @property
    def total_ke(self) -> float:
        return self.translational_ke + self.rotational_ke

    def to_dict(self) -> dict[str, Any]:
        return {
            "momentum": vec3_to_list(self.momentum),
            "angular_momentum": vec3_to_list(self.angular_momentum),
            "translational_ke": self.translational_ke,
            "rotational_ke": self.rotational_ke,
            "total_ke": self.total_ke,
        }


@dataclass
class CollisionEvent:
    event_index: int
    time: float
    body_i: int
    body_j: int
    point: Vec3
    normal: Vec3
    penetration: float
    num_contacts_before_average: int
    contact_mode: str
    pre_relative_normal_speed: float
    impulse_magnitude: float
    contact_details: list[ContactDetail]
    pre_quantities: SystemQuantities
    post_quantities: SystemQuantities

    def to_dict(self) -> dict[str, Any]:
        return {
            "event_index": self.event_index,
            "time": self.time,
            "body_i": self.body_i,
            "body_j": self.body_j,
            "point": vec3_to_list(self.point),
            "normal": vec3_to_list(self.normal),
            "penetration": self.penetration,
            "num_contacts_before_average": self.num_contacts_before_average,
            "contact_mode": self.contact_mode,
            "pre_relative_normal_speed": self.pre_relative_normal_speed,
            "impulse_magnitude": self.impulse_magnitude,
            "contact_details": [d.to_dict() for d in self.contact_details],
            "pre_quantities": self.pre_quantities.to_dict(),
            "post_quantities": self.post_quantities.to_dict(),
        }


class Particle(NodePath):
    def __init__(self, name: str = "particle"):
        super().__init__(name)
        self.vel = Vec3(0.0, 0.0, 0.0)
        self.inverseMass = 1.0
        self.radius = 1.0
        self.collisionRadius = 1.8

    @property
    def pos(self) -> Vec3:
        return self.getPos()

    @pos.setter
    def pos(self, value: Vec3) -> None:
        self.setPos(value)


class RigidBody2D(Particle):
    def __init__(self, body_id: int):
        super().__init__(name=f"body_{body_id}")
        self.body_id = body_id
        self.theta = 0.0
        self.omega = 0.0
        self.inverseMomentOfInertia = self.inverseMass / (2.0 / 3.0 * self.radius**2)

    @property
    def mass(self) -> float:
        if self.inverseMass <= EPS:
            return float("inf")
        return 1.0 / self.inverseMass

    @property
    def moment_of_inertia(self) -> float:
        if self.inverseMomentOfInertia <= EPS:
            return float("inf")
        return 1.0 / self.inverseMomentOfInertia

    def world_square_vertices_xz(self) -> list[tuple[float, float]]:
        local_vertices = [(-1.0, -1.0), (-1.0, 1.0), (1.0, 1.0), (1.0, -1.0)]
        world: list[tuple[float, float]] = []
        for lx, lz in local_vertices:
            rx, rz = rotate_xz(lx * self.radius, lz * self.radius, self.theta)
            world.append((self.pos.x + rx, self.pos.z + rz))
        return world


class CollisionEngine2D:
    def __init__(self, scenario_id: int = 0, restitution: float = 1.0):
        self.scenario_id = scenario_id
        self.eCoeffRestitution = restitution
        self.g = 0.0
        self.time = 0.0
        self.bodies = self._build_scenario(scenario_id)
        self.events: list[CollisionEvent] = []
        self._active_contacts: dict[tuple[int, int], bool] = {}

    def _build_scenario(self, scenario_id: int) -> list[RigidBody2D]:
        bodies: list[RigidBody2D] = []
        for i in range(2):
            b = RigidBody2D(i)
            b.pos = Vec3(-6.0 + 17.0 * i, 25.0, i - 0.5)
            b.vel = Vec3(2.0 - i * 6.0, 0.0, 0.0)
            bodies.append(b)

        if scenario_id == 1:
            bodies[0].theta = deg_to_rad(30.0)
        elif scenario_id == 2:
            bodies[0].theta = deg_to_rad(60.0)
            bodies[1].theta = deg_to_rad(30.0)
        elif scenario_id == 3:
            bodies[0].omega = -5.0
            bodies[1].vel = Vec3(0.0, 0.0, 0.0)
            bodies[1].pos = Vec3(-1.9, 25.0, 1.9)
        elif scenario_id == 4:
            bodies[0].omega = 3.0
            bodies[1].theta = deg_to_rad(-10.0)
        elif scenario_id != 0:
            raise ValueError(f"Scenario {scenario_id} is invalid; expected 0..4")

        return bodies

    def system_quantities(self) -> SystemQuantities:
        total_p = Vec3(0.0, 0.0, 0.0)
        total_l = Vec3(0.0, 0.0, 0.0)
        k_trans = 0.0
        k_rot = 0.0
        for b in self.bodies:
            m = b.mass
            lin_mom = b.vel * m
            total_p += lin_mom
            total_l += b.pos.cross(lin_mom)
            total_l += Vec3(0.0, b.moment_of_inertia * b.omega, 0.0)
            k_trans += 0.5 * m * b.vel.dot(b.vel)
            k_rot += 0.5 * b.moment_of_inertia * b.omega * b.omega
        return SystemQuantities(
            momentum=total_p,
            angular_momentum=total_l,
            translational_ke=k_trans,
            rotational_ke=k_rot,
        )

    def _sat_axes(self, verts: list[tuple[float, float]]) -> list[tuple[float, float]]:
        axes: list[tuple[float, float]] = []
        for i in range(len(verts)):
            a = verts[i]
            b = verts[(i + 1) % len(verts)]
            edge = vec2_sub(b, a)
            axis = vec2_norm((-edge[1], edge[0]))
            axes.append(axis)
        return axes

    def _project_poly(
        self, verts: list[tuple[float, float]], axis: tuple[float, float]
    ) -> tuple[float, float]:
        values = [vec2_dot(v, axis) for v in verts]
        return (min(values), max(values))

    def _classify_contact(
        self,
        point_xz: tuple[float, float],
        body_i: RigidBody2D,
        body_j: RigidBody2D,
        tol: float = 1e-5,
    ) -> tuple[str, Optional[int], Optional[int]]:
        verts_i = body_i.world_square_vertices_xz()
        verts_j = body_j.world_square_vertices_xz()

        on_i_vertex = any(vec2_len(vec2_sub(point_xz, v)) <= tol for v in verts_i)
        on_j_vertex = any(vec2_len(vec2_sub(point_xz, v)) <= tol for v in verts_j)

        if on_i_vertex and not on_j_vertex:
            return (
                f"body {body_i.body_id} corner against body {body_j.body_id} face",
                body_i.body_id,
                body_j.body_id,
            )
        if on_j_vertex and not on_i_vertex:
            return (
                f"body {body_j.body_id} corner against body {body_i.body_id} face",
                body_j.body_id,
                body_i.body_id,
            )
        if on_i_vertex and on_j_vertex:
            return ("corner-corner contact", None, None)
        return ("face-face (edge-edge) contact", None, None)

    def collide_square_square(self, body_i: RigidBody2D, body_j: RigidBody2D) -> Optional[CollisionManifold]:
        verts_i = body_i.world_square_vertices_xz()
        verts_j = body_j.world_square_vertices_xz()

        axes = self._sat_axes(verts_i) + self._sat_axes(verts_j)

        min_overlap = float("inf")
        best_axis = (1.0, 0.0)

        for axis in axes:
            i_min, i_max = self._project_poly(verts_i, axis)
            j_min, j_max = self._project_poly(verts_j, axis)
            overlap = min(i_max, j_max) - max(i_min, j_min)
            if overlap < 0.0:
                return None
            if overlap < min_overlap:
                min_overlap = overlap
                best_axis = axis

        center_i = (body_i.pos.x, body_i.pos.z)
        center_j = (body_j.pos.x, body_j.pos.z)
        center_delta = vec2_sub(center_j, center_i)
        if vec2_dot(center_delta, best_axis) < 0.0:
            best_axis = (-best_axis[0], -best_axis[1])

        contacts_xz: list[tuple[float, float]] = []

        for v in verts_i:
            if point_in_convex_polygon(v, verts_j):
                contacts_xz.append(v)
        for v in verts_j:
            if point_in_convex_polygon(v, verts_i):
                contacts_xz.append(v)

        for i in range(4):
            ai = verts_i[i]
            bi = verts_i[(i + 1) % 4]
            for j in range(4):
                aj = verts_j[j]
                bj = verts_j[(j + 1) % 4]
                hit = segment_intersection(ai, bi, aj, bj)
                if hit is not None:
                    contacts_xz.append(hit)

        contacts_xz = dedupe_points(contacts_xz, tol=1e-6)
        if not contacts_xz:
            avg_center = vec2_avg([center_i, center_j])
            contacts_xz = [avg_center]
        else:
            contacts_xz = reduce_contact_points(
                points=contacts_xz,
                normal=best_axis,
                verts_i=verts_i,
                verts_j=verts_j,
                penetration=min_overlap,
            )

        averaged_x, averaged_z = vec2_avg(contacts_xz)
        averaged_point = Vec3(averaged_x, 0.5 * (body_i.pos.y + body_j.pos.y), averaged_z)
        normal = Vec3(best_axis[0], 0.0, best_axis[1])

        contact_details: list[ContactDetail] = []
        for cx, cz in contacts_xz:
            classification, corner_body, face_body = self._classify_contact((cx, cz), body_i, body_j)
            contact_details.append(
                ContactDetail(
                    point=Vec3(cx, averaged_point.y, cz),
                    classification=classification,
                    corner_body=corner_body,
                    face_body=face_body,
                )
            )

        return CollisionManifold(
            normal=normal,
            penetration=min_overlap,
            averaged_point=averaged_point,
            contacts=contact_details,
        )

    def _advance_kinematics(self, dt: float) -> None:
        for b in self.bodies:
            a = Vec3(0.0, 0.0, -self.g)
            b.vel += a * dt
            b.pos += b.vel * dt
            b.theta += b.omega * dt

    def _resolve_pair(self, body_i: RigidBody2D, body_j: RigidBody2D) -> Optional[CollisionEvent]:
        pair_key = tuple(sorted((body_i.body_id, body_j.body_id)))
        delta = body_j.pos - body_i.pos
        if delta.length() > body_i.collisionRadius + body_j.collisionRadius:
            self._active_contacts[pair_key] = False
            return None

        manifold = self.collide_square_square(body_i, body_j)
        if manifold is None:
            self._active_contacts[pair_key] = False
            return None

        if self._active_contacts.get(pair_key, False):
            return None

        n = manifold.normal.normalized()
        rp = manifold.averaged_point

        ri = rp - body_i.pos
        rj = rp - body_j.pos

        omega_i = Vec3(0.0, body_i.omega, 0.0)
        omega_j = Vec3(0.0, body_j.omega, 0.0)
        vi_contact = body_i.vel + omega_i.cross(ri)
        vj_contact = body_j.vel + omega_j.cross(rj)

        rel = vj_contact - vi_contact
        vn = rel.dot(n)
        if vn >= -1e-6:
            self._active_contacts[pair_key] = True
            return None

        pre_q = self.system_quantities()

        ri_cross_n = ri.cross(n).y
        rj_cross_n = rj.cross(n).y
        denom = (
            body_i.inverseMass
            + body_j.inverseMass
            + ri_cross_n * ri_cross_n * body_i.inverseMomentOfInertia
            + rj_cross_n * rj_cross_n * body_j.inverseMomentOfInertia
        )
        if denom <= EPS:
            return None

        impulse_mag = -(1.0 + self.eCoeffRestitution) * vn / denom
        impulse = n * impulse_mag

        body_i.vel -= impulse * body_i.inverseMass
        body_j.vel += impulse * body_j.inverseMass

        body_i.omega -= ri.cross(impulse).y * body_i.inverseMomentOfInertia
        body_j.omega += rj.cross(impulse).y * body_j.inverseMomentOfInertia

        post_q = self.system_quantities()

        inv_mass_sum = body_i.inverseMass + body_j.inverseMass
        if inv_mass_sum > EPS:
            slop = 1e-6
            correction_mag = max(manifold.penetration - slop, 0.0) / inv_mass_sum
            correction = n * correction_mag
            body_i.pos -= correction * body_i.inverseMass
            body_j.pos += correction * body_j.inverseMass

        event = CollisionEvent(
            event_index=len(self.events),
            time=self.time,
            body_i=body_i.body_id,
            body_j=body_j.body_id,
            point=manifold.averaged_point,
            normal=n,
            penetration=manifold.penetration,
            num_contacts_before_average=len(manifold.contacts),
            contact_mode=(
                "single-contact"
                if len(manifold.contacts) == 1
                else "simultaneous-contacts-averaged"
            ),
            pre_relative_normal_speed=vn,
            impulse_magnitude=impulse_mag,
            contact_details=manifold.contacts,
            pre_quantities=pre_q,
            post_quantities=post_q,
        )
        self.events.append(event)
        self._active_contacts[pair_key] = True
        return event

    def step(self, dt: float) -> list[CollisionEvent]:
        self._advance_kinematics(dt)
        self.time += dt
        step_events: list[CollisionEvent] = []
        for i, bi in enumerate(self.bodies):
            for bj in self.bodies[i + 1 :]:
                event = self._resolve_pair(bi, bj)
                if event is not None:
                    step_events.append(event)
        return step_events

    def run(
        self,
        dt: float = 1.0 / 240.0,
        max_time: float = 6.0,
        stop_after_first_collision: bool = False,
        settle_time_after_first: float = 0.5,
    ) -> None:
        first_collision_time: Optional[float] = None
        max_steps = int(max_time / dt) + 2

        for _ in range(max_steps):
            step_events = self.step(dt)
            if step_events and first_collision_time is None:
                first_collision_time = step_events[0].time
            if stop_after_first_collision and first_collision_time is not None:
                if self.time >= first_collision_time + settle_time_after_first:
                    break
            if self.time >= max_time:
                break

    def describe_scenario(self) -> str:
        descriptions = {
            0: "Default setup with no initial rotation.",
            1: "Body 0 starts rotated by 30 degrees.",
            2: "Body 0 starts at 60 degrees and body 1 starts at 30 degrees.",
            3: "Body 0 starts spinning at -5 rad/s, while body 1 is moved to x=-1.9, z=1.9 and starts from rest.",
            4: "Body 0 starts spinning at +3 rad/s, and body 1 starts at -10 degrees.",
        }
        return descriptions[self.scenario_id]


class CollisionApp(ShowBase):
    def __init__(self, engine: CollisionEngine2D, dt: float, max_time: float):
        super().__init__()

        self.engine = engine
        self.fixed_dt = dt
        self.max_time = max_time
        self.accumulator = 0.0
        self.event_count_seen = 0

        self.disableMouse()
        self.camera.setPos(0.0, -35.0, 8.0)
        self.camera.lookAt(0.0, 25.0, 0.0)

        ambient = AmbientLight("ambient")
        ambient.setColor((0.55, 0.55, 0.55, 1.0))
        ambient_np = self.render.attachNewNode(ambient)
        self.render.setLight(ambient_np)

        sun = DirectionalLight("sun")
        sun.setColor((0.8, 0.8, 0.8, 1.0))
        sun_np = self.render.attachNewNode(sun)
        sun_np.setPos(10.0, -20.0, 10.0)
        sun_np.lookAt(0.0, 25.0, 0.0)
        self.render.setLight(sun_np)

        cube_path_candidates = [
            Path(__file__).resolve().parent / "Cube.egg",
            Path(__file__).resolve().parent.parent / "participation_3" / "Cube.egg",
        ]
        cube_model = None
        for candidate in cube_path_candidates:
            if candidate.exists():
                cube_model = self.loader.loadModel(str(candidate))
                break
        if cube_model is None:
            cube_model = self.loader.loadModel("Cube")
        cube_model.setScale(0.6)

        self.body_nodes = []
        colors = ((0.95, 0.2, 0.15, 1.0), (0.15, 0.5, 0.95, 1.0))
        for i, body in enumerate(self.engine.bodies):
            node = self.render.attachNewNode(f"body_{i}")
            cube_model.instanceTo(node)
            node.setColor(colors[i])
            node.setScale(body.radius)
            node.setPos(body.pos)
            node.setHpr(0.0, 0.0, math.degrees(body.theta))
            self.body_nodes.append(node)

        self.taskMgr.add(self._update, "update_physics")

    def _sync_nodes(self) -> None:
        for body, node in zip(self.engine.bodies, self.body_nodes):
            node.setPos(body.pos)
            node.setHpr(0.0, 0.0, math.degrees(body.theta))

    def _print_event(self, event: CollisionEvent) -> None:
        print(
            f"[visual] collision #{event.event_index} t={event.time} "
            f"point={vec3_to_str(event.point)} normal={vec3_to_str(event.normal)} "
            f"contacts={event.num_contacts_before_average} mode={event.contact_mode}"
        )

    def _update(self, task: Task) -> int:
        frame_dt = min(globalClock.getDt(), 0.1)
        self.accumulator += frame_dt

        while self.accumulator >= self.fixed_dt:
            step_events = self.engine.step(self.fixed_dt)
            self.accumulator -= self.fixed_dt
            for event in step_events:
                self._print_event(event)

        self._sync_nodes()

        if self.engine.time >= self.max_time:
            self.userExit()
            return Task.done
        return Task.cont


def _delta_vec3(a: Vec3, b: Vec3) -> Vec3:
    return Vec3(a.x - b.x, a.y - b.y, a.z - b.z)


def _energy_comment(pre: SystemQuantities, post: SystemQuantities, restitution: float) -> str:
    d_trans = post.translational_ke - pre.translational_ke
    d_rot = post.rotational_ke - pre.rotational_ke
    d_tot = post.total_ke - pre.total_ke

    if restitution >= 0.999:
        if abs(d_tot) < 5e-6:
            base = "Total kinetic energy is effectively conserved (up to numerical precision)."
        else:
            base = "Total kinetic energy is almost conserved; the tiny drift is numerical."
    else:
        base = "Total kinetic energy drops, which is expected for an inelastic collision (e=0)."

    if d_trans < 0.0 and d_rot > 0.0:
        transfer = "Most of the lost translational energy is converted into rotation."
    elif d_trans > 0.0 and d_rot < 0.0:
        transfer = "Part of the rotational energy is converted into translational motion."
    else:
        transfer = "Translational and rotational components move in the same direction for this impact geometry."

    return f"{base} {transfer} (Delta_K_total={d_tot}, Delta_K_trans={d_trans}, Delta_K_rot={d_rot})."


def _event_wording(event: CollisionEvent) -> str:
    if event.num_contacts_before_average == 1:
        contact = event.contact_details[0]
        return (
            "This impact was a single-contact event at "
            f"{vec3_to_str(contact.point)} with classification: {contact.classification}."
        )
    details = []
    for idx, c in enumerate(event.contact_details):
        details.append(
            f"contact {idx + 1} at {vec3_to_str(c.point)} ({c.classification})"
        )
    return (
        "Multiple contacts occurred at the same instant, so they were averaged into one collision point. "
        "Before averaging, the contact set was: "
        + "; ".join(details)
        + "."
    )


def _quantities_row(q: SystemQuantities) -> list[str]:
    return [
        vec3_to_str(q.momentum),
        vec3_to_str(q.angular_momentum),
        str(q.translational_ke),
        str(q.rotational_ke),
        str(q.total_ke),
    ]


def run_case(
    scenario_id: int,
    restitution: float,
    dt: float,
    max_time: float,
    stop_after_first_collision: bool = False,
) -> dict[str, Any]:
    engine = CollisionEngine2D(scenario_id=scenario_id, restitution=restitution)
    initial_q = engine.system_quantities()
    engine.run(
        dt=dt,
        max_time=max_time,
        stop_after_first_collision=stop_after_first_collision,
        settle_time_after_first=0.5,
    )
    final_q = engine.system_quantities()

    collision_rows: list[dict[str, Any]] = [event.to_dict() for event in engine.events]

    return {
        "scenario": scenario_id,
        "scenario_description": engine.describe_scenario(),
        "restitution": restitution,
        "dt": dt,
        "max_time": max_time,
        "collision_count": len(engine.events),
        "initial_quantities": initial_q.to_dict(),
        "final_quantities": final_q.to_dict(),
        "collisions": collision_rows,
    }


def run_all_cases(dt: float, max_time: float, stop_after_first_collision: bool = False) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for restitution in (1.0, 0.0):
        for scenario_id in range(5):
            out.append(
                run_case(
                    scenario_id=scenario_id,
                    restitution=restitution,
                    dt=dt,
                    max_time=max_time,
                    stop_after_first_collision=stop_after_first_collision,
                )
            )
    return out


def write_results_json(results: list[dict[str, Any]], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)


def write_summary_csv(results: list[dict[str, Any]], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

    headers = [
        "scenario",
        "restitution",
        "collision_count",
        "first_collision_time",
        "first_collision_point",
        "first_collision_normal",
        "first_collision_mode",
        "first_collision_num_contacts",
        "pre_momentum",
        "post_momentum",
        "pre_angular_momentum",
        "post_angular_momentum",
        "pre_total_ke",
        "post_total_ke",
        "delta_total_ke",
    ]

    with output_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=headers)
        writer.writeheader()

        for result in results:
            first = result["collisions"][0] if result["collisions"] else None
            if first is None:
                row = {
                    "scenario": result["scenario"],
                    "restitution": result["restitution"],
                    "collision_count": 0,
                    "first_collision_time": "NONE",
                    "first_collision_point": "NONE",
                    "first_collision_normal": "NONE",
                    "first_collision_mode": "NONE",
                    "first_collision_num_contacts": "NONE",
                    "pre_momentum": "NONE",
                    "post_momentum": "NONE",
                    "pre_angular_momentum": "NONE",
                    "post_angular_momentum": "NONE",
                    "pre_total_ke": "NONE",
                    "post_total_ke": "NONE",
                    "delta_total_ke": "NONE",
                }
            else:
                pre = first["pre_quantities"]
                post = first["post_quantities"]
                row = {
                    "scenario": result["scenario"],
                    "restitution": result["restitution"],
                    "collision_count": result["collision_count"],
                    "first_collision_time": first["time"],
                    "first_collision_point": str(first["point"]),
                    "first_collision_normal": str(first["normal"]),
                    "first_collision_mode": first["contact_mode"],
                    "first_collision_num_contacts": first["num_contacts_before_average"],
                    "pre_momentum": str(pre["momentum"]),
                    "post_momentum": str(post["momentum"]),
                    "pre_angular_momentum": str(pre["angular_momentum"]),
                    "post_angular_momentum": str(post["angular_momentum"]),
                    "pre_total_ke": pre["total_ke"],
                    "post_total_ke": post["total_ke"],
                    "delta_total_ke": post["total_ke"] - pre["total_ke"],
                }
            writer.writerow(row)


def build_answers_pdf(results: list[dict[str, Any]], output_pdf: Path) -> None:
    from reportlab.lib import colors
    from reportlab.lib.pagesizes import letter
    from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
    from reportlab.lib.units import inch
    from reportlab.platypus import PageBreak, Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle

    output_pdf.parent.mkdir(parents=True, exist_ok=True)
    doc = SimpleDocTemplate(
        str(output_pdf),
        pagesize=letter,
        leftMargin=0.7 * inch,
        rightMargin=0.7 * inch,
        topMargin=0.7 * inch,
        bottomMargin=0.7 * inch,
        title="2VG3 HW4 Written Answers",
    )

    styles = getSampleStyleSheet()
    title_style = ParagraphStyle(
        "TitleStyle",
        parent=styles["Title"],
        fontName="Helvetica-Bold",
        fontSize=18,
        spaceAfter=12,
    )
    h_style = ParagraphStyle(
        "Heading",
        parent=styles["Heading2"],
        fontName="Helvetica-Bold",
        fontSize=12,
        spaceBefore=8,
        spaceAfter=6,
    )
    body_style = ParagraphStyle(
        "Body",
        parent=styles["BodyText"],
        fontName="Helvetica",
        fontSize=9,
        leading=12,
    )
    mono_style = ParagraphStyle(
        "Mono",
        parent=body_style,
        fontName="Courier",
        fontSize=8,
        leading=10,
    )

    elements: list[Any] = []
    elements.append(Paragraph("2VG3 Homework 4 - RigidBody Collisions in 2D", title_style))
    elements.append(Spacer(1, 10))

    for restitution in (1.0, 0.0):
        subset = [r for r in results if abs(r["restitution"] - restitution) < 1e-12]
        elements.append(
            Paragraph(
                f"Exercise 2 ({'a' if restitution == 1.0 else 'b'}) - Collision Details with e={restitution}",
                h_style,
            )
        )

        for result in sorted(subset, key=lambda x: x["scenario"]):
            elements.append(
                Paragraph(
                    f"Scenario {result['scenario']}: {result['scenario_description']}",
                    body_style,
                )
            )
            if not result["collisions"]:
                elements.append(Paragraph("No collision detected before max_time.", body_style))
                elements.append(Spacer(1, 6))
                continue

            if restitution == 0.0:
                collisions_to_report = [result["collisions"][0]]
                elements.append(
                    Paragraph(
                        "For e=0, the assignment asks for first-contact properties only, so only collision 0 is reported.",
                        body_style,
                    )
                )
            else:
                collisions_to_report = result["collisions"]

            for col in collisions_to_report:
                elements.append(
                    Paragraph(
                        f"Collision {col['event_index']}: t={col['time']}, collision point={col['point']}, "
                        f"normal={col['normal']}, contact mode={col['contact_mode']}, "
                        f"contacts before averaging={col['num_contacts_before_average']}.",
                        body_style,
                    )
                )

                narrative = _event_wording(
                    CollisionEvent(
                        event_index=col["event_index"],
                        time=col["time"],
                        body_i=col["body_i"],
                        body_j=col["body_j"],
                        point=Vec3(*col["point"]),
                        normal=Vec3(*col["normal"]),
                        penetration=col["penetration"],
                        num_contacts_before_average=col["num_contacts_before_average"],
                        contact_mode=col["contact_mode"],
                        pre_relative_normal_speed=col["pre_relative_normal_speed"],
                        impulse_magnitude=col["impulse_magnitude"],
                        contact_details=[
                            ContactDetail(
                                point=Vec3(*d["point"]),
                                classification=d["classification"],
                                corner_body=d["corner_body"],
                                face_body=d["face_body"],
                            )
                            for d in col["contact_details"]
                        ],
                        pre_quantities=SystemQuantities(
                            momentum=Vec3(*col["pre_quantities"]["momentum"]),
                            angular_momentum=Vec3(*col["pre_quantities"]["angular_momentum"]),
                            translational_ke=col["pre_quantities"]["translational_ke"],
                            rotational_ke=col["pre_quantities"]["rotational_ke"],
                        ),
                        post_quantities=SystemQuantities(
                            momentum=Vec3(*col["post_quantities"]["momentum"]),
                            angular_momentum=Vec3(*col["post_quantities"]["angular_momentum"]),
                            translational_ke=col["post_quantities"]["translational_ke"],
                            rotational_ke=col["post_quantities"]["rotational_ke"],
                        ),
                    )
                )
                elements.append(Paragraph(narrative, body_style))

            elements.append(Spacer(1, 6))

    elements.append(PageBreak())
    elements.append(Paragraph("Exercise 3 - Momentum, Angular Momentum, and Energy", h_style))
    elements.append(
        Paragraph(
            "The table below lists values immediately before and immediately after the first collision in each scenario.",
            body_style,
        )
    )

    for restitution in (1.0, 0.0):
        subset = [r for r in results if abs(r["restitution"] - restitution) < 1e-12]
        elements.append(Spacer(1, 8))
        elements.append(Paragraph(f"Restitution e={restitution}", h_style))

        table_data = [
            [
                "Scenario",
                "Stage",
                "P",
                "L",
                "K_trans",
                "K_rot",
                "K_total",
            ]
        ]

        commentary: list[str] = []
        for result in sorted(subset, key=lambda x: x["scenario"]):
            scenario_id = result["scenario"]
            if not result["collisions"]:
                table_data.append([str(scenario_id), "no collision", "-", "-", "-", "-", "-"])
                continue

            first = result["collisions"][0]
            pre = first["pre_quantities"]
            post = first["post_quantities"]

            pre_q = SystemQuantities(
                momentum=Vec3(*pre["momentum"]),
                angular_momentum=Vec3(*pre["angular_momentum"]),
                translational_ke=pre["translational_ke"],
                rotational_ke=pre["rotational_ke"],
            )
            post_q = SystemQuantities(
                momentum=Vec3(*post["momentum"]),
                angular_momentum=Vec3(*post["angular_momentum"]),
                translational_ke=post["translational_ke"],
                rotational_ke=post["rotational_ke"],
            )

            table_data.append([str(scenario_id), "pre", *_quantities_row(pre_q)])
            table_data.append([str(scenario_id), "post", *_quantities_row(post_q)])

            delta_p = _delta_vec3(post_q.momentum, pre_q.momentum)
            delta_l = _delta_vec3(post_q.angular_momentum, pre_q.angular_momentum)
            commentary.append(
                f"Scenario {scenario_id}: momentum change Delta_P={vec3_to_str(delta_p)}, "
                f"angular momentum change Delta_L={vec3_to_str(delta_l)}. "
                f"{_energy_comment(pre_q, post_q, restitution)}"
            )

        table = Table(table_data, repeatRows=1)
        table.setStyle(
            TableStyle(
                [
                    ("BACKGROUND", (0, 0), (-1, 0), colors.lightgrey),
                    ("TEXTCOLOR", (0, 0), (-1, 0), colors.black),
                    ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
                    ("FONTNAME", (0, 1), (-1, -1), "Courier"),
                    ("FONTSIZE", (0, 0), (-1, -1), 7),
                    ("GRID", (0, 0), (-1, -1), 0.25, colors.grey),
                    ("VALIGN", (0, 0), (-1, -1), "TOP"),
                    ("ALIGN", (0, 0), (1, -1), "CENTER"),
                ]
            )
        )
        elements.append(table)
        elements.append(Spacer(1, 6))

        for c in commentary:
            elements.append(Paragraph(c, body_style))

    doc.build(elements)


def print_console_summary(results: list[dict[str, Any]]) -> None:
    for result in results:
        scenario_id = result["scenario"]
        e = result["restitution"]
        print(f"scenario={scenario_id} e={e} collisions={result['collision_count']}")
        if result["collisions"]:
            first = result["collisions"][0]
            print(
                f"  first collision: t={first['time']} point={first['point']} normal={first['normal']} "
                f"contacts={first['num_contacts_before_average']} mode={first['contact_mode']}"
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="2VG3 HW4 rigid body 2D square collision simulation")
    parser.add_argument("--scenario", type=int, default=scenario, help="Scenario index [0..4]")
    parser.add_argument("--restitution", type=float, default=1.0, help="Coefficient of restitution")
    parser.add_argument("--dt", type=float, default=1.0 / 240.0, help="Fixed timestep")
    parser.add_argument("--max-time", type=float, default=6.0, help="Maximum simulation time")
    parser.add_argument("--run-all", action="store_true", help="Run scenarios 0..4 for e=1 and e=0")
    parser.add_argument(
        "--stop-after-first-collision",
        action="store_true",
        help="Stop after first collision plus short settle period",
    )
    parser.add_argument("--headless", action="store_true", help="Headless mode for single scenario")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "output",
        help="Directory for JSON/CSV outputs",
    )
    parser.add_argument("--write-json", action="store_true", help="Write JSON results")
    parser.add_argument("--write-csv", action="store_true", help="Write CSV summary")
    parser.add_argument(
        "--generate-answers-pdf",
        action="store_true",
        help="Generate HW4 answers PDF from results",
    )
    parser.add_argument(
        "--answers-pdf-path",
        type=Path,
        default=Path(__file__).resolve().parent / "HW4_answers.pdf",
        help="Output path for answers PDF",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.run_all:
        results = run_all_cases(
            dt=args.dt,
            max_time=args.max_time,
            stop_after_first_collision=args.stop_after_first_collision,
        )
        print_console_summary(results)

        if args.write_json or args.generate_answers_pdf:
            write_results_json(results, args.output_dir / "hw4_results.json")
            print(f"Wrote {args.output_dir / 'hw4_results.json'}")

        if args.write_csv or args.generate_answers_pdf:
            write_summary_csv(results, args.output_dir / "hw4_summary.csv")
            print(f"Wrote {args.output_dir / 'hw4_summary.csv'}")

        if args.generate_answers_pdf:
            build_answers_pdf(results, args.answers_pdf_path)
            print(f"Wrote {args.answers_pdf_path}")
        return

    if args.headless:
        result = run_case(
            scenario_id=args.scenario,
            restitution=args.restitution,
            dt=args.dt,
            max_time=args.max_time,
            stop_after_first_collision=args.stop_after_first_collision,
        )
        print_console_summary([result])
        return

    engine = CollisionEngine2D(scenario_id=args.scenario, restitution=args.restitution)
    app = CollisionApp(engine=engine, dt=args.dt, max_time=args.max_time)
    app.run()


if __name__ == "__main__":
    main()
