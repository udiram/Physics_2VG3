from __future__ import annotations

import argparse
import copy
import json
import math
import os
from dataclasses import dataclass

from direct.gui.OnscreenText import OnscreenText
from direct.showbase.ShowBase import ShowBase
from panda3d.core import (
    AmbientLight,
    DirectionalLight,
    Filename,
    Geom,
    GeomNode,
    GeomTriangles,
    GeomVertexData,
    GeomVertexFormat,
    GeomVertexWriter,
    Mat3,
    OrthographicLens,
    TextNode,
    Vec3,
    loadPrcFileData,
)

from rigidbody2d_collision_mod import collideShapes, markerClean, markerInit, safe_normalized
from rigidbody2d_mod import RigidBody2D


EPSILON = 1.0e-8
DEFAULT_SCENARIO = 5
DEFAULT_DT = 1.0 / 120.0
DEFAULT_ITERATIONS = 120
DEFAULT_BIAS_BETA = 0.15
DEFAULT_BIAS_SLOP = 0.005
DEFAULT_BIAS_RESTING_THRESHOLD = 0.5


@dataclass
class ContactConstraint:
    pi: RigidBody2D
    pj: RigidBody2D
    xclose: Vec3
    normal: Vec3
    depth: float
    normal_impulse: float
    friction_impulse: Vec3
    target_normal_velocity: float


def square_inverse_moi(inverse_mass: float, radius: float) -> float:
    if inverse_mass <= 0.0:
        return 0.0
    return inverse_mass / ((2.0 / 3.0) * radius * radius)


def vector_to_list(vec: Vec3) -> list[float]:
    return [float(vec.x), float(vec.y), float(vec.z)]


def world_tangent(theta: float) -> Vec3:
    return Vec3(math.cos(theta), 0.0, -math.sin(theta))


def world_normal(theta: float) -> Vec3:
    return Vec3(math.sin(theta), 0.0, math.cos(theta))


def build_ramp_node(
    surface_origin: Vec3,
    tangent: Vec3,
    normal: Vec3,
    half_length: float,
    thickness: float,
    color: tuple[float, float, float, float],
):
    p0 = surface_origin - tangent * half_length
    p1 = surface_origin + tangent * half_length
    p2 = p1 - normal * thickness
    p3 = p0 - normal * thickness

    vertex_data = GeomVertexData("seq-ramp", GeomVertexFormat.getV3c4(), Geom.UHStatic)
    vertex_writer = GeomVertexWriter(vertex_data, "vertex")
    color_writer = GeomVertexWriter(vertex_data, "color")
    for point in (p0, p1, p2, p3):
        vertex_writer.addData3(point)
        color_writer.addData4(*color)

    triangles = GeomTriangles(Geom.UHStatic)
    triangles.addVertices(0, 1, 2)
    triangles.addVertices(0, 2, 3)
    geom = Geom(vertex_data)
    geom.addPrimitive(triangles)

    node = GeomNode("seq-ramp")
    node.addGeom(geom)
    return node


def build_box_node(center: Vec3, radius: float, angle: float, color: tuple[float, float, float, float]):
    square = [
        Vec3(-1.0, 0.0, -1.0),
        Vec3(-1.0, 0.0, 1.0),
        Vec3(1.0, 0.0, 1.0),
        Vec3(1.0, 0.0, -1.0),
    ]
    transform = Mat3().rotateMatNormaxis(math.degrees(angle), Vec3(0.0, 1.0, 0.0))
    vertices = [center + transform.xform(vertex * radius) for vertex in square]

    vertex_data = GeomVertexData("seq-box", GeomVertexFormat.getV3c4(), Geom.UHStatic)
    vertex_writer = GeomVertexWriter(vertex_data, "vertex")
    color_writer = GeomVertexWriter(vertex_data, "color")
    for vertex in vertices:
        vertex_writer.addData3(vertex)
        color_writer.addData4(*color)

    triangles = GeomTriangles(Geom.UHStatic)
    triangles.addVertices(0, 1, 2)
    triangles.addVertices(0, 2, 3)
    geom = Geom(vertex_data)
    geom.addPrimitive(triangles)

    node = GeomNode("seq-box")
    node.addGeom(geom)
    return node


class SimpleScene(ShowBase):
    def __init__(self, args: argparse.Namespace):
        super().__init__(windowType="offscreen" if args.headless else None)

        self.headless = args.headless
        self.scenario = args.scenario
        self.dt = args.dt
        self.nSolverIterations = args.iterations
        self.biasBeta = args.bias_beta
        self.biasSlop = args.bias_slop
        self.biasOnElastic = args.bias_on_elastic
        self.biasRestingThreshold = DEFAULT_BIAS_RESTING_THRESHOLD
        self.capture_every = max(1, args.capture_every)
        self.capture_frame_dir = args.frame_dir

        self.g = 10.0
        self.eCoeffRestitution = 0.05
        self.muFrictionStatic = 0.5
        self.muFrictionKinetic = 0.5

        self.fixPos = True
        self.fixVel = False
        self.t = 0.0
        self.iFrame = 0
        self.last_contact_count = 0
        self.showMarkers = False

        lens = OrthographicLens()
        lens.setFilmSize(8.0, 6.0)
        lens.setNearFar(-100.0, 100.0)
        self.cam.node().setLens(lens)
        self.cam.setPos(0.0, -24.0, 0.0)
        self.cam.lookAt(0.0, 0.0, -0.7)
        self.setBackgroundColor(0.97, 0.97, 0.985, 1.0)

        ambient = AmbientLight("ambient")
        ambient.setColor((0.55, 0.55, 0.58, 1.0))
        ambient_np = render.attachNewNode(ambient)
        render.setLight(ambient_np)

        dlight = DirectionalLight("dlight")
        dlight.setColor((0.85, 0.85, 0.88, 1.0))
        dlight_np = render.attachNewNode(dlight)
        dlight_np.setPos(10.0, -18.0, 18.0)
        dlight_np.lookAt(0.0, 0.0, -0.5)
        render.setLight(dlight_np)

        markerInit(self, "models/misc/sphere")

        self.RigidBody2Ds: list[RigidBody2D] = []
        self.plane_body: RigidBody2D | None = None
        self.plane_visual = None
        self.plane_visual_color = (0.26, 0.27, 0.31, 1.0)
        self.info_text: OnscreenText | None = None

        self.setup_scenario()

        if not self.headless:
            self.info_text = OnscreenText(
                text="",
                parent=self.aspect2d,
                pos=(-1.30, 0.90),
                align=TextNode.ALeft,
                scale=0.045,
                fg=(0.95, 0.95, 0.95, 1.0),
                mayChange=True,
            )
            self.update_overlay()
            self.accept("space", self.change_scenario, [1])
            self.accept("n", self.change_scenario, [1])
            self.accept("p", self.change_scenario, [-1])
            self.accept("r", self.setup_scenario)
            self.accept("escape", self.userExit)
            self.taskMgr.add(self.updateRigidBody2Ds, "updateRigidBody2Ds")

    def change_scenario(self, delta: int) -> None:
        self.scenario = (self.scenario + delta) % 7
        self.setup_scenario()

    def setup_scenario(self) -> None:
        for body in self.RigidBody2Ds:
            if getattr(body, "visual_node", None) is not None:
                body.visual_node.removeNode()
            body.removeNode()
        self.RigidBody2Ds = []
        if self.plane_visual is not None:
            self.plane_visual.removeNode()
            self.plane_visual = None

        if hasattr(self, "markers"):
            for marker in self.markers:
                marker.removeNode()
            self.markers = []

        self.t = 0.0
        self.iFrame = 0
        self.last_contact_count = 0
        self.plane_body = None
        incline_angle = math.asin(0.1)
        angle = incline_angle if self.scenario in (0, 2, 5, 6) else 0.0
        plane_velocity = Vec3(-1.0, 0.0, 0.0) if self.scenario == 6 else Vec3(0.0, 0.0, 0.0)
        plane_radius = 2.0
        plane_pos = Vec3(0.2, 0.0, -plane_radius - 1.0)
        impact_velocity = Vec3(-math.sin(angle), 0.0, -math.cos(angle))
        straight_down = Vec3(0.0, 0.0, -1.0)

        if self.scenario == 0:
            self.eCoeffRestitution = 0.05
            self.muFrictionStatic = 0.5
            self.muFrictionKinetic = 0.5
            self.plane_visual_color = (0.94, 0.87, 0.18, 1.0)
            self.add_plane(angle, radius=plane_radius, vel=Vec3(0.0, 0.0, 0.0), pos=plane_pos)
            self.add_body(
                name="body_1",
                radius=0.5,
                mass=1.0,
                pos=Vec3(0.0, 0.0, 0.5),
                vel=impact_velocity,
                theta=angle,
                color=Vec3(0.88, 0.28, 0.18),
                gravity=True,
            )

        elif self.scenario == 1:
            self.eCoeffRestitution = 0.0
            self.muFrictionStatic = 0.5
            self.muFrictionKinetic = 0.5
            self.plane_visual_color = (0.94, 0.87, 0.18, 1.0)
            self.add_plane(angle, radius=plane_radius, vel=Vec3(0.0, 0.0, 0.0), pos=plane_pos)
            self.add_body(
                name="body_1",
                radius=0.5,
                mass=1.0,
                pos=Vec3(0.0, 0.0, 0.5),
                vel=straight_down,
                theta=angle,
                color=Vec3(0.20, 0.55, 0.90),
                gravity=False,
            )

        elif self.scenario in (2, 3, 4):
            self.eCoeffRestitution = 1.0
            self.muFrictionStatic = 0.0 if self.scenario == 4 else 0.5
            self.muFrictionKinetic = 0.0 if self.scenario == 4 else 0.5
            self.plane_visual_color = (0.94, 0.87, 0.18, 1.0)
            self.add_plane(angle, radius=plane_radius, vel=Vec3(0.0, 0.0, 0.0), pos=plane_pos)
            self.add_body(
                name="body_1",
                radius=0.5,
                mass=1.0,
                pos=Vec3(0.0, 0.0, 0.5),
                vel=impact_velocity if self.scenario == 2 else straight_down,
                theta=angle,
                color=Vec3(0.20, 0.55, 0.90),
                gravity=(self.scenario == 4),
            )

        elif self.scenario in (5, 6):
            self.eCoeffRestitution = 0.05
            self.muFrictionStatic = 0.2
            self.muFrictionKinetic = 0.2
            self.plane_visual_color = (0.94, 0.87, 0.18, 1.0)
            self.add_plane(angle, radius=plane_radius, vel=plane_velocity, pos=plane_pos)

            stack_colors = [
                Vec3(0.88, 0.28, 0.18),
                Vec3(0.05, 0.05, 0.05),
                Vec3(0.20, 0.90, 0.22),
                Vec3(0.94, 0.10, 0.94),
                Vec3(0.96, 0.96, 0.96),
            ]
            falling_velocity = impact_velocity + plane_velocity
            self.add_body(
                name="body_1",
                radius=0.5,
                mass=1.0,
                pos=Vec3(0.0, 0.0, 0.5),
                vel=falling_velocity,
                theta=angle,
                color=stack_colors[0],
                gravity=True,
            )
            for i, color in zip(range(2, 6), stack_colors[1:]):
                radius = 0.3 - i * 0.01
                self.add_body(
                    name=f"body_{i}",
                    radius=radius,
                    mass=0.2,
                    pos=Vec3(-0.1 + (i % 3) * 0.05, 0.0, 2.0 + (i - 2) * 0.5),
                    vel=falling_velocity,
                    theta=angle,
                    color=color,
                    gravity=True,
                )

        else:
            raise ValueError(f"Unsupported scenario {self.scenario}. Expected 0..6.")

        self.update_all_visuals()
        self.update_overlay()

    def add_body(
        self,
        name: str,
        radius: float,
        mass: float | None,
        pos: Vec3,
        vel: Vec3,
        theta: float,
        color: Vec3,
        gravity: bool,
        with_box_visual: bool = True,
    ) -> RigidBody2D:
        body = RigidBody2D(name)
        body.radius = radius
        body.shape = "square"
        body.collisionRadius = 1.8 * radius
        body.inverseMass = 0.0 if mass is None or mass <= 0.0 else 1.0 / mass
        body.inverseMomentOfInertia = square_inverse_moi(body.inverseMass, radius)
        body.vel = Vec3(vel)
        body.vold = Vec3(vel)
        body.rold = Vec3(pos)
        body.theta = theta
        body.thetaold = theta
        body.omega = 0.0
        body.doGravity = gravity
        body.setPos(pos)
        body.reparentTo(render)
        if with_box_visual:
            body.visual_color = (float(color.x), float(color.y), float(color.z), 1.0)
            body.visual_node = render.attachNewNode(build_box_node(body.getPos(), radius, theta, body.visual_color))
            body.visual_node.setTwoSided(True)
        else:
            body.visual_color = None
            body.visual_node = None
        self.RigidBody2Ds.append(body)
        return body

    def add_plane(self, angle: float, radius: float, vel: Vec3, pos: Vec3 | None = None) -> None:
        plane_pos = Vec3(0.0, 0.0, -radius - 1.0) if pos is None else Vec3(pos)
        self.plane_body = self.add_body(
            name="plane",
            radius=radius,
            mass=None,
            pos=plane_pos,
            vel=vel,
            theta=angle,
            color=Vec3(0.35, 0.36, 0.42),
            gravity=False,
            with_box_visual=False,
        )
        self.update_plane_visual()

    def add_box_on_plane(
        self,
        radius: float,
        mass: float,
        tangent_offset: float,
        gap: float,
        vel: Vec3,
        color: Vec3,
        gravity: bool,
        angle: float,
    ) -> None:
        if self.plane_body is None:
            raise RuntimeError("Plane body must be added before dynamic boxes.")

        tangent = world_tangent(angle)
        normal = world_normal(angle)
        plane_surface_origin = self.plane_body.getPos() + normal * self.plane_body.radius
        pos = plane_surface_origin + tangent * tangent_offset + normal * (radius + gap)
        self.add_body(
            name=f"body_{len(self.RigidBody2Ds)}",
            radius=radius,
            mass=mass,
            pos=pos,
            vel=vel,
            theta=angle,
            color=color,
            gravity=gravity,
        )

    def updateRigidBody2Ds(self, task):
        self.step_physics()
        self.update_overlay()
        return task.cont

    def update_overlay(self) -> None:
        if self.info_text is None:
            return
        label = {
            0: "Stop on incline",
            1: "Stop on flat plane",
            2: "Bounce on incline",
            3: "Bounce on flat plane",
            4: "Repeated bounce",
            5: "Stack on incline",
            6: "Moving-frame stack",
        }[self.scenario]
        self.info_text.setText(
            f"scenario={self.scenario} {label}\n"
            f"iterations={self.nSolverIterations}  dt={self.dt:.5f}\n"
            f"e={self.eCoeffRestitution:.2f}  mu=({self.muFrictionStatic:.2f}, {self.muFrictionKinetic:.2f})\n"
            f"contacts={self.last_contact_count}  t={self.t:.2f}\n"
            "keys: space/n next, p prev, r reset"
        )

    def update_all_visuals(self) -> None:
        self.update_plane_visual()
        for body in self.RigidBody2Ds:
            self.apply_visual_transform(body)
        self.update_camera()

    def update_plane_visual(self) -> None:
        if self.plane_body is None:
            return
        if self.plane_visual is not None:
            self.plane_visual.removeNode()

        tangent = world_tangent(self.plane_body.theta)
        normal = world_normal(self.plane_body.theta)
        surface_origin = self.plane_body.getPos() + normal * self.plane_body.radius
        self.plane_visual = render.attachNewNode(
            build_ramp_node(
                surface_origin=surface_origin,
                tangent=tangent,
                normal=normal,
                half_length=self.plane_body.radius,
                thickness=0.45,
                color=self.plane_visual_color,
            )
        )
        self.plane_visual.setTwoSided(True)

    def update_camera(self) -> None:
        focus_x = self.plane_body.getPos().x if self.plane_body is not None else 0.0
        self.cam.setPos(focus_x, -24.0, 0.0)
        self.cam.lookAt(focus_x, 0.0, -0.35)

    def apply_visual_transform(self, body: RigidBody2D) -> None:
        if body is self.plane_body:
            return
        if getattr(body, "visual_node", None) is not None:
            body.visual_node.removeNode()
        body.visual_node = render.attachNewNode(build_box_node(body.getPos(), body.radius, body.theta, body.visual_color))
        body.visual_node.setTwoSided(True)

    def contact_velocity(self, body: RigidBody2D, offset: Vec3) -> Vec3:
        return body.vel + Vec3(0.0, body.omega, 0.0).cross(offset)

    def apply_delta_impulse(self, body: RigidBody2D, impulse: Vec3, offset: Vec3, sign: float) -> None:
        if body.inverseMass <= 0.0:
            return
        body.vel += impulse * (sign * body.inverseMass)
        body.omega += offset.cross(impulse).y * sign * body.inverseMomentOfInertia

    def collect_constraints(self) -> list[ContactConstraint]:
        constraints = []
        for i, pi in enumerate(self.RigidBody2Ds):
            xi = pi.getPos()
            for pj in self.RigidBody2Ds[i + 1 :]:
                xj = pj.getPos()
                if (xj - xi).length() >= pi.collisionRadius + pj.collisionRadius:
                    continue

                for depth, xclose, normal in collideShapes(self, pi, pj):
                    if xclose is None or normal is None:
                        continue

                    normal = safe_normalized(normal)
                    dxi = xclose - xi
                    dxj = xclose - xj
                    ui = self.contact_velocity(pi, dxi)
                    uj = self.contact_velocity(pj, dxj)
                    uij = uj - ui
                    unormal = uij.dot(normal)
                    # Bias helps resting contacts separate a bit, but it should not pump energy into clean elastic bounces.
                    use_bias = self.biasOnElastic or self.eCoeffRestitution < 0.95
                    apply_resting_bias = abs(unormal) <= self.biasRestingThreshold
                    v_bias = 0.0
                    if use_bias and apply_resting_bias and depth >= self.biasSlop:
                        v_bias = depth * self.biasBeta / self.dt

                    constraints.append(
                        ContactConstraint(
                            pi=pi,
                            pj=pj,
                            xclose=Vec3(xclose),
                            normal=Vec3(normal),
                            depth=float(depth),
                            normal_impulse=0.0,
                            friction_impulse=Vec3(0.0, 0.0, 0.0),
                            target_normal_velocity=-self.eCoeffRestitution * unormal + v_bias,
                        )
                    )

        return constraints

    def solve_constraint(self, constraint: ContactConstraint) -> None:
        pi = constraint.pi
        pj = constraint.pj
        xi = pi.getPos()
        xj = pj.getPos()
        dxi = constraint.xclose - xi
        dxj = constraint.xclose - xj

        ui = self.contact_velocity(pi, dxi)
        uj = self.contact_velocity(pj, dxj)
        uij = uj - ui
        unormal = uij.dot(constraint.normal)

        rni = dxi.cross(constraint.normal)
        rnj = dxj.cross(constraint.normal)
        inverse_mass_normal = (
            pi.inverseMass
            + pj.inverseMass
            + rni.dot(rni) * pi.inverseMomentOfInertia
            + rnj.dot(rnj) * pj.inverseMomentOfInertia
        )
        if inverse_mass_normal <= EPSILON:
            return

        dp_normal_delta = (unormal - constraint.target_normal_velocity) / inverse_mass_normal
        dp_normal_new = constraint.normal_impulse + dp_normal_delta
        if dp_normal_new > 0.0:
            dp_normal_new = 0.0
        dp_normal_delta = dp_normal_new - constraint.normal_impulse
        constraint.normal_impulse = dp_normal_new

        friction_vector = uij - constraint.normal * unormal
        dp_friction_delta = Vec3(0.0, 0.0, 0.0)
        if friction_vector.length() > EPSILON:
            friction_dir = safe_normalized(friction_vector)
            rnfi = dxi.cross(friction_dir)
            rnfj = dxj.cross(friction_dir)
            inverse_mass_friction = (
                pi.inverseMass
                + pj.inverseMass
                + rnfi.dot(rnfi) * pi.inverseMomentOfInertia
                + rnfj.dot(rnfj) * pj.inverseMomentOfInertia
            )
            if inverse_mass_friction > EPSILON:
                dp_friction_trial = friction_vector / inverse_mass_friction
                dp_friction_new = constraint.friction_impulse + dp_friction_trial
                static_limit = self.muFrictionStatic * abs(dp_normal_new)
                friction_magnitude = dp_friction_new.length()
                if friction_magnitude > static_limit and friction_magnitude > EPSILON:
                    dp_friction_new *= self.muFrictionKinetic * abs(dp_normal_new) / friction_magnitude
                dp_friction_delta = dp_friction_new - constraint.friction_impulse
                constraint.friction_impulse = dp_friction_new

        total_impulse = constraint.normal * dp_normal_delta + dp_friction_delta
        self.apply_delta_impulse(pi, total_impulse, dxi, +1.0)
        self.apply_delta_impulse(pj, total_impulse, dxj, -1.0)

    def step_physics(self) -> None:
        markerClean(self)

        for body in self.RigidBody2Ds:
            acceleration = Vec3(0.0, 0.0, 0.0)
            if body.inverseMass > 0.0 and body.doGravity:
                acceleration = Vec3(0.0, 0.0, -self.g)

            body.vold = copy.deepcopy(body.vel)
            body.rold = Vec3(body.getPos())
            body.thetaold = body.theta
            body.vel += acceleration * self.dt
            body.setPos(body.getPos() + body.vel * self.dt)
            body.theta += body.omega * self.dt

        constraints = self.collect_constraints()
        self.last_contact_count = len(constraints)

        if constraints:
            for _ in range(self.nSolverIterations):
                for constraint in constraints:
                    self.solve_constraint(constraint)

            if self.fixPos:
                for body in self.RigidBody2Ds:
                    body.setPos(body.rold + (body.vel + body.vold) * (0.5 * self.dt))
                    body.theta = body.thetaold + body.omega * self.dt

        self.t += self.dt
        self.iFrame += 1
        self.update_all_visuals()

    def run_for_duration(self, duration: float) -> dict:
        nsteps = max(1, int(round(duration / self.dt)))
        for _ in range(nsteps):
            self.step_physics()
            if self.headless:
                self.graphicsEngine.renderFrame()
                self.maybe_capture_frame()
        return self.build_summary()

    def maybe_capture_frame(self) -> None:
        if not self.capture_frame_dir:
            return
        if self.iFrame % self.capture_every != 0:
            return
        os.makedirs(self.capture_frame_dir, exist_ok=True)
        frame_path = os.path.join(self.capture_frame_dir, f"frame_{self.iFrame:05d}.png")
        self.win.saveScreenshot(Filename.fromOsSpecific(frame_path))

    def save_screenshot(self, path: str) -> None:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.graphicsEngine.renderFrame()
        self.win.saveScreenshot(Filename.fromOsSpecific(path))

    def build_summary(self) -> dict:
        return {
            "scenario": self.scenario,
            "time": self.t,
            "dt": self.dt,
            "iterations": self.nSolverIterations,
            "restitution": self.eCoeffRestitution,
            "mu_static": self.muFrictionStatic,
            "mu_kinetic": self.muFrictionKinetic,
            "last_contact_count": self.last_contact_count,
            "bodies": [
                {
                    "name": body.getName(),
                    "pos": vector_to_list(body.getPos()),
                    "vel": vector_to_list(body.vel),
                    "theta": float(body.theta),
                    "omega": float(body.omega),
                }
                for body in self.RigidBody2Ds
            ],
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Homework 6 sequential impulse rigid-body solver.")
    parser.add_argument("--scenario", type=int, default=DEFAULT_SCENARIO, help="Scenario id (0..6).")
    parser.add_argument("--iterations", type=int, default=DEFAULT_ITERATIONS, help="Sequential impulse iterations.")
    parser.add_argument("--dt", type=float, default=DEFAULT_DT, help="Fixed timestep.")
    parser.add_argument("--duration", type=float, default=4.0, help="Headless run duration in seconds.")
    parser.add_argument("--bias-beta", type=float, default=DEFAULT_BIAS_BETA, help="Velocity bias factor.")
    parser.add_argument(
        "--bias-slop",
        type=float,
        default=DEFAULT_BIAS_SLOP,
        help="Disable velocity bias below this penetration depth.",
    )
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


def configure_prc(headless: bool) -> None:
    loadPrcFileData("", "audio-library-name null")
    loadPrcFileData("", "sync-video false")
    loadPrcFileData("", "show-frame-rate-meter false")
    loadPrcFileData("", "win-size 1280 960")
    if headless:
        loadPrcFileData("", "window-type offscreen")


def main() -> None:
    args = parse_args()
    configure_prc(args.headless)
    scene = SimpleScene(args)
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
