import argparse
import json
import math
import os
from dataclasses import asdict, dataclass

from direct.gui.OnscreenText import OnscreenText
from panda3d.core import (
    Geom,
    GeomNode,
    GeomTriangles,
    GeomVertexData,
    GeomVertexFormat,
    GeomVertexWriter,
    PerspectiveLens,
    TextNode,
    Vec3,
)

from rigidbody2d_collision_mod import collideShapes, markerClean
from rigidbody2d_friction import FrictionScene, build_ramp_node, configure_prc, vector_to_list
from rigidbody2d_mod import RigidBody2D


EPSILON = 1.0e-8
VISUAL_CLEARANCE = 0.02
TUBE_SCALE = 1.3 / 1.41421356237
START_S = -2.0
RACE_LANE_SPACING = 5.0


@dataclass
class ScenarioSpec:
    key: str
    label: str
    inertia_factor: float
    shape: str
    model: str
    color: tuple[float, float, float, float]
    friction: float


SCENARIOS: dict[str, ScenarioSpec] = {
    "hollow": ScenarioSpec("hollow", "Hollow Cylinder", 1.0, "cylinder", "tube", (0.88, 0.26, 0.18, 1.0), 0.5),
    "solid": ScenarioSpec("solid", "Solid Cylinder", 0.5, "cylinder", "disk", (0.18, 0.52, 0.90, 1.0), 0.5),
    "sphere": ScenarioSpec("sphere", "Sphere", 0.4, "cylinder", "sphere", (0.20, 0.74, 0.42, 1.0), 0.5),
    "slider": ScenarioSpec("slider", "Sliding Particle", 0.0, "square", "cube", (0.94, 0.82, 0.18, 1.0), 0.0),
}


@dataclass
class RollingConfig:
    scenario: str = "hollow"
    angle_deg: float = 13.0
    dt: float = 1.0 / 240.0
    gravity: float = 10.0
    restitution: float = 0.02
    mu_static: float = 0.5
    mu_kinetic: float = 0.5
    duration: float = 5.0
    headless: bool = False
    screenshot_path: str | None = None
    report_path: str | None = None
    solver_iterations: int = 12
    position_percent: float = 0.9
    position_slop: float = 5.0e-4
    enable_rest_snap: bool = False
    rest_speed_threshold: float = 0.0
    rest_omega_threshold: float = 0.0
    rest_contact_frames: int = 0
    capture_history: bool = True


def build_disk_node(radius: float, color: tuple[float, float, float, float], segments: int = 48):
    vertex_data = GeomVertexData("disk", GeomVertexFormat.getV3c4(), Geom.UHStatic)
    vertex_writer = GeomVertexWriter(vertex_data, "vertex")
    color_writer = GeomVertexWriter(vertex_data, "color")

    vertex_writer.addData3(0.0, 0.0, 0.0)
    color_writer.addData4(*color)
    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        vertex_writer.addData3(radius * math.cos(angle), 0.0, radius * math.sin(angle))
        color_writer.addData4(*color)

    triangles = GeomTriangles(Geom.UHStatic)
    for i in range(segments):
        triangles.addVertices(0, 1 + i, 1 + ((i + 1) % segments))

    geom = Geom(vertex_data)
    geom.addPrimitive(triangles)
    node = GeomNode("disk")
    node.addGeom(geom)
    return node


def build_cylinder_node(
    radius: float,
    half_depth: float,
    color: tuple[float, float, float, float],
    segments: int = 48,
):
    vertex_data = GeomVertexData("solid-cylinder", GeomVertexFormat.getV3n3c4(), Geom.UHStatic)
    vertex_writer = GeomVertexWriter(vertex_data, "vertex")
    normal_writer = GeomVertexWriter(vertex_data, "normal")
    color_writer = GeomVertexWriter(vertex_data, "color")

    side_top_indices = []
    side_bottom_indices = []
    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        x = radius * math.cos(angle)
        z = radius * math.sin(angle)
        normal = Vec3(math.cos(angle), 0.0, math.sin(angle))

        side_top_indices.append(vertex_data.getNumRows())
        vertex_writer.addData3(x, half_depth, z)
        normal_writer.addData3(normal)
        color_writer.addData4(*color)

        side_bottom_indices.append(vertex_data.getNumRows())
        vertex_writer.addData3(x, -half_depth, z)
        normal_writer.addData3(normal)
        color_writer.addData4(*color)

    front_center = vertex_data.getNumRows()
    vertex_writer.addData3(0.0, -half_depth, 0.0)
    normal_writer.addData3(0.0, -1.0, 0.0)
    color_writer.addData4(*color)

    front_ring_indices = []
    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        front_ring_indices.append(vertex_data.getNumRows())
        vertex_writer.addData3(radius * math.cos(angle), -half_depth, radius * math.sin(angle))
        normal_writer.addData3(0.0, -1.0, 0.0)
        color_writer.addData4(*color)

    back_center = vertex_data.getNumRows()
    vertex_writer.addData3(0.0, half_depth, 0.0)
    normal_writer.addData3(0.0, 1.0, 0.0)
    color_writer.addData4(*color)

    back_ring_indices = []
    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        back_ring_indices.append(vertex_data.getNumRows())
        vertex_writer.addData3(radius * math.cos(angle), half_depth, radius * math.sin(angle))
        normal_writer.addData3(0.0, 1.0, 0.0)
        color_writer.addData4(*color)

    triangles = GeomTriangles(Geom.UHStatic)
    for i in range(segments):
        ni = (i + 1) % segments
        top0 = side_top_indices[i]
        top1 = side_top_indices[ni]
        bottom0 = side_bottom_indices[i]
        bottom1 = side_bottom_indices[ni]
        triangles.addVertices(top0, bottom0, bottom1)
        triangles.addVertices(top0, bottom1, top1)

        triangles.addVertices(front_center, front_ring_indices[ni], front_ring_indices[i])
        triangles.addVertices(back_center, back_ring_indices[i], back_ring_indices[ni])

    geom = Geom(vertex_data)
    geom.addPrimitive(triangles)
    node = GeomNode("solid-cylinder")
    node.addGeom(geom)
    return node


def ensure_models(scene: FrictionScene) -> None:
    if hasattr(scene, "tube_template"):
        return

    scene.tube_template = loader.loadModel("Tube")
    sphere_path = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "HW2", "sphere.egg.pz"))
    scene.sphere_template = loader.loadModel(sphere_path) if os.path.exists(sphere_path) else None


def analytic_exit_speed_for_spec(spec: ScenarioSpec, gravity: float, incline_theta: float, start_s: float, exit_s: float) -> float:
    travel = exit_s - start_s
    height_drop = travel * math.sin(incline_theta)
    if spec.shape == "square":
        return math.sqrt(max(0.0, 2.0 * gravity * height_drop))
    return math.sqrt(max(0.0, 2.0 * gravity * height_drop / (1.0 + spec.inertia_factor)))


def interpolate_exit_state(history: list[dict], exit_s: float) -> dict | None:
    if len(history) < 2:
        return None

    prev = history[-2]
    curr = history[-1]
    s0 = prev["tangent_position"]
    s1 = curr["tangent_position"]
    if s0 > exit_s or s1 < exit_s:
        return None

    ds = s1 - s0
    alpha = 0.0 if abs(ds) <= EPSILON else (exit_s - s0) / ds
    alpha = max(0.0, min(1.0, alpha))
    time = prev["time"] + alpha * (curr["time"] - prev["time"])

    pos0 = Vec3(*prev["position"])
    pos1 = Vec3(*curr["position"])
    vel0 = Vec3(*prev["velocity"])
    vel1 = Vec3(*curr["velocity"])
    position = pos0 * (1.0 - alpha) + pos1 * alpha
    velocity = vel0 * (1.0 - alpha) + vel1 * alpha
    return {
        "time": time,
        "position": position,
        "velocity": velocity,
        "speed": float(velocity.length()),
    }


class RollingScene(FrictionScene):
    def __init__(self, config: RollingConfig):
        self.spec = SCENARIOS[config.scenario]
        self.start_s = START_S
        self.exit_s = 4.0
        super().__init__(config)

    def maybe_snap_to_rest(self) -> None:
        return

    def setup_visuals(self, body: RigidBody2D) -> None:
        body.visual_anchor = body.attachNewNode("visual-anchor")
        body.spin_node = body.visual_anchor.attachNewNode("spin-node")
        body.visual_node = body.spin_node.attachNewNode("visual-node")

        if self.spec.model == "tube":
            self.tube_template.instanceTo(body.visual_node)
            body.visual_node.setScale(TUBE_SCALE)
            body.visual_node.setHpr(0.0, 90.0, 0.0)
            body.visual_node.setPos(0.0, 0.1, 0.0)
            body.visual_node.setColor(*self.spec.color)
        elif self.spec.model == "disk":
            solid = body.visual_node.attachNewNode(build_cylinder_node(body.radius, 0.18, self.spec.color))
            solid.setTwoSided(True)
        elif self.spec.model == "sphere":
            if self.sphere_template is not None:
                self.sphere_template.instanceTo(body.visual_node)
                body.visual_node.setScale(body.radius)
                body.visual_node.setColor(*self.spec.color)
            else:
                body.visual_node.attachNewNode(build_disk_node(body.radius, self.spec.color))
        else:
            self.cube.instanceTo(body.visual_node)
            body.visual_node.setScale(body.radius)
            body.visual_node.setColor(*self.spec.color)

        if self.spec.shape == "cylinder":
            marker = body.visual_node.attachNewNode("spin-marker")
            self.cube.instanceTo(marker)
            marker.setScale(0.15)
            marker.setPos(0.72, 0.0, 0.0)
            marker.setColor(0.04, 0.04, 0.04, 1.0)

    def setup_incline_scene(self) -> None:
        ensure_models(self)

        for body in self.RigidBody2Ds:
            body.removeNode()

        self.RigidBody2Ds = []
        for index in range(2):
            body = RigidBody2D(f"body-{index}")
            body.reparentTo(render)
            body.vel = Vec3(0.0, 0.0, 0.0)
            body.vold = Vec3(0.0, 0.0, 0.0)
            body.rold = Vec3(0.0, 0.0, 0.0)
            body.doGravity = False
            self.RigidBody2Ds.append(body)

        self.dynamic_body = self.RigidBody2Ds[0]
        self.plane_body = self.RigidBody2Ds[1]

        self.dynamic_body.radius = 1.0
        self.dynamic_body.shape = self.spec.shape
        if self.dynamic_body.shape == "cylinder":
            self.dynamic_body.collisionRadius = 1.1 * self.dynamic_body.radius
        else:
            self.dynamic_body.collisionRadius = 1.8 * self.dynamic_body.radius
        self.dynamic_body.inverseMass = 1.0
        if self.dynamic_body.shape == "cylinder":
            self.dynamic_body.inverseMomentOfInertia = (
                self.dynamic_body.inverseMass / (self.spec.inertia_factor * self.dynamic_body.radius ** 2)
            )
            self.dynamic_body.theta = 0.0
        else:
            self.dynamic_body.inverseMomentOfInertia = 0.0
            self.dynamic_body.theta = self.incline_theta
        self.dynamic_body.omega = 0.0
        self.dynamic_body.doGravity = True
        self.setup_visuals(self.dynamic_body)

        self.plane_body.radius = 4.0
        self.plane_body.shape = "square"
        self.plane_body.collisionRadius = 1.8 * self.plane_body.radius
        self.plane_body.inverseMass = 0.0
        self.plane_body.inverseMomentOfInertia = 0.0
        self.plane_body.theta = self.incline_theta
        self.plane_body.setPos(0.0, 3.0, -self.plane_body.radius - 1.0)

        if self.visual_ramp is not None:
            self.visual_ramp.removeNode()
        surface_origin = self.plane_body.getPos() + self.incline_normal * self.plane_body.radius
        self.exit_s = self.plane_body.radius
        self.visual_ramp = render.attachNewNode(
            build_ramp_node(
                surface_origin,
                self.incline_tangent,
                self.incline_normal,
                self.plane_body.radius,
                0.45,
                (0.22, 0.23, 0.29, 1.0),
            )
        )
        self.visual_ramp.setTwoSided(True)

        self.dynamic_body.setPos(
            surface_origin
            + self.incline_tangent * self.start_s
            + self.incline_normal * (self.dynamic_body.radius + 0.01)
        )
        self.dynamic_body.vel = Vec3(0.0, 0.0, 0.0)
        self.apply_visual_transform(self.dynamic_body)

    def apply_visual_transform(self, body: RigidBody2D) -> None:
        if body is not self.dynamic_body or not hasattr(body, "visual_anchor"):
            return

        body.visual_anchor.setPos(self.incline_normal * VISUAL_CLEARANCE)
        if self.spec.shape == "cylinder":
            body.spin_node.setR(math.degrees(body.theta))
        else:
            body.visual_node.setHpr(90.0, math.degrees(body.theta), 0.0)

    def analytic_exit_speed(self) -> float:
        return analytic_exit_speed_for_spec(self.spec, self.g, self.incline_theta, self.start_s, self.exit_s)

    def exit_state(self) -> dict | None:
        if len(self.history) < 2:
            return None

        for prev, curr in zip(self.history, self.history[1:]):
            s0 = prev["tangent_position"]
            s1 = curr["tangent_position"]
            if s0 > self.exit_s or s1 < self.exit_s:
                continue
            ds = s1 - s0
            if abs(ds) <= EPSILON:
                alpha = 0.0
            else:
                alpha = (self.exit_s - s0) / ds
            alpha = max(0.0, min(1.0, alpha))
            time = prev["time"] + alpha * (curr["time"] - prev["time"])

            pos0 = Vec3(*prev["position"])
            pos1 = Vec3(*curr["position"])
            vel0 = Vec3(*prev["velocity"])
            vel1 = Vec3(*curr["velocity"])
            position = pos0 * (1.0 - alpha) + pos1 * alpha
            velocity = vel0 * (1.0 - alpha) + vel1 * alpha
            return {
                "time": time,
                "position": position,
                "velocity": velocity,
                "speed": float(velocity.length()),
            }
        return None

    def build_summary(self) -> dict:
        exit_state = self.exit_state()
        analytic_speed = self.analytic_exit_speed()
        measured_speed = None if exit_state is None else exit_state["speed"]
        percent_error = None
        if measured_speed is not None and analytic_speed > EPSILON:
            percent_error = 100.0 * (measured_speed - analytic_speed) / analytic_speed

        return {
            "config": asdict(self.sim_config),
            "scenario": self.spec.key,
            "label": self.spec.label,
            "analytic_exit_speed": analytic_speed,
            "measured_exit_speed": measured_speed,
            "current_speed": float(self.dynamic_body.vel.length()),
            "percent_error": percent_error,
            "exit_time": None if exit_state is None else exit_state["time"],
            "exit_position": None if exit_state is None else vector_to_list(exit_state["position"]),
            "final_position": vector_to_list(self.dynamic_body.getPos()),
            "final_velocity": vector_to_list(self.dynamic_body.vel),
            "final_theta": float(self.dynamic_body.theta),
            "final_omega": float(self.dynamic_body.omega),
            "history": self.history,
        }


class RollingRaceScene(FrictionScene):
    def __init__(self, config: RollingConfig):
        self.lanes: list[dict] = []
        self.visual_ramps: list = []
        self.status_text: OnscreenText | None = None
        self.start_s = START_S
        super().__init__(config)

        lens = PerspectiveLens()
        lens.setFov(50.0)
        lens.setNearFar(1.0, 100.0)
        self.cam.node().setLens(lens)
        self.cam.setPos(-6.0, -22.0, 6.0)
        self.cam.lookAt(1.0, 0.0, -2.0)

        if not config.headless:
            self.status_text = OnscreenText(
                text="",
                parent=self.aspect2d,
                pos=(-1.28, 0.88),
                align=TextNode.ALeft,
                scale=0.045,
                fg=(0.95, 0.95, 0.95, 1.0),
                mayChange=True,
            )
            self.accept("r", self.reset_race)
            self.accept("escape", self.userExit)
            self.update_status_text()

    def reset_race(self) -> None:
        self.t = 0.0
        self.iFrame = 0
        self.last_contact_count = 0
        self.setup_incline_scene()
        self.update_status_text()

    def maybe_snap_to_rest(self) -> None:
        return

    def setup_lane_visuals(self, body: RigidBody2D, spec: ScenarioSpec) -> None:
        body.race_spec = spec
        body.visual_anchor = body.attachNewNode("visual-anchor")
        body.spin_node = body.visual_anchor.attachNewNode("spin-node")
        body.visual_node = body.spin_node.attachNewNode("visual-node")

        if spec.model == "tube":
            self.tube_template.instanceTo(body.visual_node)
            body.visual_node.setScale(TUBE_SCALE)
            body.visual_node.setHpr(0.0, 90.0, 0.0)
            body.visual_node.setPos(0.0, 0.1, 0.0)
            body.visual_node.setColor(*spec.color)
        elif spec.model == "disk":
            solid = body.visual_node.attachNewNode(build_cylinder_node(body.radius, 0.18, spec.color))
            solid.setTwoSided(True)
        elif spec.model == "sphere":
            if self.sphere_template is not None:
                self.sphere_template.instanceTo(body.visual_node)
                body.visual_node.setScale(body.radius)
                body.visual_node.setColor(*spec.color)
            else:
                body.visual_node.attachNewNode(build_disk_node(body.radius, spec.color))
        else:
            self.cube.instanceTo(body.visual_node)
            body.visual_node.setScale(body.radius)
            body.visual_node.setColor(*spec.color)

        if spec.shape == "cylinder":
            marker = body.visual_node.attachNewNode("spin-marker")
            self.cube.instanceTo(marker)
            marker.setScale(0.15)
            marker.setPos(0.72, 0.0, 0.0)
            marker.setColor(0.04, 0.04, 0.04, 1.0)

    def setup_incline_scene(self) -> None:
        ensure_models(self)

        for body in self.RigidBody2Ds:
            body.removeNode()
        for ramp in self.visual_ramps:
            ramp.removeNode()

        self.RigidBody2Ds = []
        self.dynamic_body = None
        self.plane_body = None
        self.lanes = []
        self.visual_ramps = []

        y_offsets = [
            (index - 0.5 * (len(SCENARIOS) - 1)) * RACE_LANE_SPACING for index in range(len(SCENARIOS))
        ]
        for y_offset, (_, spec) in zip(y_offsets, SCENARIOS.items()):
            dynamic_body = RigidBody2D(f"{spec.key}-dynamic")
            plane_body = RigidBody2D(f"{spec.key}-plane")
            for body in (dynamic_body, plane_body):
                body.reparentTo(render)
                body.vel = Vec3(0.0, 0.0, 0.0)
                body.vold = Vec3(0.0, 0.0, 0.0)
                body.rold = Vec3(0.0, 0.0, 0.0)
                body.doGravity = False
                self.RigidBody2Ds.append(body)

            dynamic_body.radius = 1.0
            dynamic_body.shape = spec.shape
            dynamic_body.collisionRadius = 1.1 * dynamic_body.radius if spec.shape == "cylinder" else 1.8 * dynamic_body.radius
            dynamic_body.inverseMass = 1.0
            dynamic_body.mu_static = spec.friction
            dynamic_body.mu_kinetic = spec.friction
            if spec.shape == "cylinder":
                dynamic_body.inverseMomentOfInertia = dynamic_body.inverseMass / (spec.inertia_factor * dynamic_body.radius ** 2)
                dynamic_body.theta = 0.0
            else:
                dynamic_body.inverseMomentOfInertia = 0.0
                dynamic_body.theta = self.incline_theta
            dynamic_body.omega = 0.0
            dynamic_body.doGravity = True
            self.setup_lane_visuals(dynamic_body, spec)

            plane_body.radius = 4.0
            plane_body.shape = "square"
            plane_body.collisionRadius = 1.8 * plane_body.radius
            plane_body.inverseMass = 0.0
            plane_body.inverseMomentOfInertia = 0.0
            plane_body.mu_static = spec.friction
            plane_body.mu_kinetic = spec.friction
            plane_body.theta = self.incline_theta
            plane_body.setPos(0.0, y_offset, -plane_body.radius - 1.0)

            surface_origin = plane_body.getPos() + self.incline_normal * plane_body.radius
            ramp = render.attachNewNode(
                build_ramp_node(
                    surface_origin,
                    self.incline_tangent,
                    self.incline_normal,
                    plane_body.radius,
                    0.45,
                    (0.22, 0.23, 0.29, 1.0),
                )
            )
            ramp.setTwoSided(True)
            self.visual_ramps.append(ramp)

            dynamic_body.setPos(
                surface_origin
                + self.incline_tangent * self.start_s
                + self.incline_normal * (dynamic_body.radius + 0.01)
            )
            dynamic_body.vel = Vec3(0.0, 0.0, 0.0)
            self.apply_visual_transform(dynamic_body)

            self.lanes.append(
                {
                    "spec": spec,
                    "dynamic_body": dynamic_body,
                    "plane_body": plane_body,
                    "exit_s": plane_body.radius,
                    "history": [],
                    "exit_time": None,
                    "measured_exit_speed": None,
                }
            )

    def apply_visual_transform(self, body: RigidBody2D) -> None:
        spec = getattr(body, "race_spec", None)
        if spec is None or not hasattr(body, "visual_anchor"):
            return

        body.visual_anchor.setPos(self.incline_normal * VISUAL_CLEARANCE)
        if spec.shape == "cylinder":
            body.spin_node.setR(math.degrees(body.theta))
        else:
            body.visual_node.setHpr(90.0, math.degrees(body.theta), 0.0)

    def lane_tangent_position(self, lane: dict) -> float:
        plane_body = lane["plane_body"]
        dynamic_body = lane["dynamic_body"]
        surface_origin = plane_body.getPos() + self.incline_normal * plane_body.radius
        return float((dynamic_body.getPos() - surface_origin).dot(self.incline_tangent))

    def update_status_text(self) -> None:
        if self.status_text is None:
            return

        lines = ["Race View", "r: reset    esc: quit", ""]
        ranked_lanes = sorted(self.lanes, key=self.lane_tangent_position, reverse=True)
        for index, lane in enumerate(ranked_lanes, start=1):
            spec = lane["spec"]
            if lane["measured_exit_speed"] is None:
                state = f"s={self.lane_tangent_position(lane):5.2f}"
            else:
                state = f"v={lane['measured_exit_speed']:.3f}"
            lines.append(f"{index}. {spec.label}: {state}")

        if all(lane["measured_exit_speed"] is not None for lane in self.lanes):
            lines.extend(["", "Finished"])
            for lane in sorted(self.lanes, key=lambda item: item["measured_exit_speed"], reverse=True):
                lines.append(f"{lane['spec'].label}: {lane['measured_exit_speed']:.3f}")

        self.status_text.setText("\n".join(lines))

    def step_physics(self) -> None:
        markerClean(self)

        for body in self.RigidBody2Ds:
            if body.inverseMass <= 0.0:
                continue
            acceleration = Vec3(0.0, 0.0, -self.g) if body.doGravity else Vec3(0.0, 0.0, 0.0)
            body.vold = Vec3(body.vel)
            body.rold = Vec3(body.getPos())
            body.vel += acceleration * self.dt
            body.setPos(body.getPos() + body.vel * self.dt)
            body.theta += body.omega * self.dt

        self.last_contact_count = 0
        for _ in range(self.sim_config.solver_iterations):
            for index, pi in enumerate(self.RigidBody2Ds):
                for pj in self.RigidBody2Ds[index + 1 :]:
                    if (pj.getPos() - pi.getPos()).length() >= pi.collisionRadius + pj.collisionRadius:
                        continue
                    contacts = collideShapes(self, pi, pj)
                    for depth, xclose, normal in contacts:
                        if xclose is None or normal is None:
                            continue
                        saved_mu_static = self.muFrictionStatic
                        saved_mu_kinetic = self.muFrictionKinetic
                        self.muFrictionStatic = min(
                            getattr(pi, "mu_static", saved_mu_static), getattr(pj, "mu_static", saved_mu_static)
                        )
                        self.muFrictionKinetic = min(
                            getattr(pi, "mu_kinetic", saved_mu_kinetic), getattr(pj, "mu_kinetic", saved_mu_kinetic)
                        )
                        resolved = self.resolve_contact(pi, pj, float(depth), xclose, normal.normalized())
                        self.muFrictionStatic = saved_mu_static
                        self.muFrictionKinetic = saved_mu_kinetic
                        if resolved:
                            self.last_contact_count += 1

        self.t += self.dt
        self.iFrame += 1

        for lane in self.lanes:
            dynamic_body = lane["dynamic_body"]
            plane_body = lane["plane_body"]
            surface_origin = plane_body.getPos() + self.incline_normal * plane_body.radius
            rel = dynamic_body.getPos() - surface_origin
            lane["history"].append(
                {
                    "time": self.t,
                    "position": vector_to_list(dynamic_body.getPos()),
                    "velocity": vector_to_list(dynamic_body.vel),
                    "theta": float(dynamic_body.theta),
                    "omega": float(dynamic_body.omega),
                    "tangent_position": float(rel.dot(self.incline_tangent)),
                    "normal_offset": float(rel.dot(self.incline_normal) - dynamic_body.radius),
                    "tangent_velocity": float(dynamic_body.vel.dot(self.incline_tangent)),
                }
            )

            if lane["measured_exit_speed"] is None:
                exit_state = interpolate_exit_state(lane["history"], lane["exit_s"])
                if exit_state is not None:
                    lane["exit_time"] = exit_state["time"]
                    lane["measured_exit_speed"] = exit_state["speed"]
                    print(
                        f"{lane['spec'].label}: exit speed {lane['measured_exit_speed']:.6f} at t={lane['exit_time']:.3f}s"
                    )

        for body in self.RigidBody2Ds:
            self.apply_visual_transform(body)
        self.update_status_text()

    def build_summary(self) -> dict:
        results = []
        for lane in self.lanes:
            spec = lane["spec"]
            analytic_speed = analytic_exit_speed_for_spec(spec, self.g, self.incline_theta, self.start_s, lane["exit_s"])
            percent_error = None
            if lane["measured_exit_speed"] is not None and analytic_speed > EPSILON:
                percent_error = 100.0 * (lane["measured_exit_speed"] - analytic_speed) / analytic_speed
            results.append(
                {
                    "scenario": spec.key,
                    "label": spec.label,
                    "analytic_exit_speed": analytic_speed,
                    "measured_exit_speed": lane["measured_exit_speed"],
                    "percent_error": percent_error,
                    "exit_time": lane["exit_time"],
                }
            )

        return {
            "angle_deg": self.sim_config.angle_deg,
            "results": results,
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Rolling bodies on an incline for 2VG3 Homework 5.")
    parser.add_argument(
        "--scenario",
        choices=tuple(SCENARIOS.keys()),
        default="hollow",
        help="Which rolling/sliding body to simulate.",
    )
    parser.add_argument("--race", action="store_true", help="Visualize all four race cases together.")
    parser.add_argument("--headless", action="store_true", help="Run without an on-screen window.")
    parser.add_argument("--duration", type=float, default=5.0, help="Simulation duration in seconds.")
    parser.add_argument("--angle", type=float, default=13.0, help="Incline angle in degrees.")
    parser.add_argument("--dt", type=float, default=1.0 / 240.0, help="Fixed time step.")
    parser.add_argument("--screenshot", type=str, default=None, help="Optional PNG path for the final frame.")
    parser.add_argument("--report", type=str, default=None, help="Optional JSON path for the run report.")
    parser.add_argument(
        "--race-report",
        type=str,
        default=None,
        help="Optional JSON path. When set, all four scenarios are run and recorded.",
    )
    return parser.parse_args()


def make_config(args: argparse.Namespace, scenario: str | None = None) -> RollingConfig:
    spec = SCENARIOS[args.scenario if scenario is None else scenario]
    return RollingConfig(
        scenario=spec.key,
        angle_deg=args.angle,
        dt=args.dt,
        duration=args.duration,
        headless=args.headless,
        screenshot_path=args.screenshot,
        report_path=args.report,
        mu_static=spec.friction,
        mu_kinetic=spec.friction,
    )


def run_scene(scene_type, config: RollingConfig) -> dict:
    configure_prc(config.headless)
    scene = scene_type(config)
    try:
        return scene.run_for_duration(config.duration)
    finally:
        scene.destroy()


def run_race(args: argparse.Namespace, output_path: str) -> None:
    results = []
    for scenario in SCENARIOS:
        config = make_config(args, scenario=scenario)
        config.headless = True
        config.screenshot_path = None
        config.report_path = None
        summary = run_scene(RollingScene, config)
        results.append(
            {
                "scenario": summary["scenario"],
                "label": summary["label"],
                "analytic_exit_speed": summary["analytic_exit_speed"],
                "measured_exit_speed": summary["measured_exit_speed"],
                "percent_error": summary["percent_error"],
                "exit_time": summary["exit_time"],
            }
        )

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as handle:
        json.dump({"angle_deg": args.angle, "results": results}, handle, indent=2)


def main() -> None:
    args = parse_args()
    if args.race_report:
        run_race(args, args.race_report)
        return

    if args.race:
        config = RollingConfig(
            angle_deg=args.angle,
            dt=args.dt,
            duration=args.duration,
            headless=args.headless,
            screenshot_path=args.screenshot,
            report_path=args.report,
        )
        if not args.headless:
            config.headless = False
            config.screenshot_path = None
            config.report_path = None
            config.dt = args.dt / 3.0
            configure_prc(False)
            scene = RollingRaceScene(config)
            print("Interactive race demo: running at 1/3 speed. Press r to reset, close the window to quit.")
            scene.run()
            return

        summary = run_scene(RollingRaceScene, config)
        print(json.dumps(summary, indent=2))
        return

    config = make_config(args)
    if not args.headless:
        config.headless = False
        config.screenshot_path = None
        config.report_path = None
        config.dt = args.dt / 3.0
        configure_prc(False)
        scene = RollingScene(config)
        print("Interactive demo: running at 1/3 speed.")
        scene.run()
        return

    summary = run_scene(RollingScene, config)
    payload = {
        "scenario": summary["scenario"],
        "analytic_exit_speed": summary["analytic_exit_speed"],
        "measured_exit_speed": summary["measured_exit_speed"],
        "percent_error": summary["percent_error"],
        "exit_time": summary["exit_time"],
    }
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
