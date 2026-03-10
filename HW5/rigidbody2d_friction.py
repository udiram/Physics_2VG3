import argparse
import json
import math
import os
from dataclasses import asdict, dataclass

from direct.showbase.ShowBase import ShowBase
from panda3d.core import (
    DirectionalLight,
    Filename,
    Geom,
    GeomNode,
    GeomTriangles,
    GeomVertexData,
    GeomVertexFormat,
    GeomVertexWriter,
    OrthographicLens,
    Vec3,
    loadPrcFileData,
)

from rigidbody2d_collision_mod import collideShapes, markerClean
from rigidbody2d_mod import RigidBody2D


EPSILON = 1.0e-8


@dataclass
class FrictionConfig:
    angle_deg: float = 13.0
    dt: float = 1.0 / 120.0
    gravity: float = 10.0
    restitution: float = 0.1
    mu_static: float = 0.5
    mu_kinetic: float = 0.5
    duration: float = 6.0
    headless: bool = False
    screenshot_path: str | None = None
    report_path: str | None = None
    solver_iterations: int = 1
    position_percent: float = 0.85
    position_slop: float = 1.0e-3
    enable_rest_snap: bool = False
    tangential_static_position_percent: float = 0.0
    tangential_sliding_position_percent: float = 0.0
    rest_speed_threshold: float = 0.02
    rest_omega_threshold: float = 0.03
    rest_contact_frames: int = 18
    disable_gravity_when_resting: bool = False
    capture_history: bool = True


def configure_prc(headless: bool) -> None:
    loadPrcFileData("", "audio-library-name null")
    loadPrcFileData("", "sync-video false")
    loadPrcFileData("", "show-frame-rate-meter false")
    loadPrcFileData("", "win-size 1280 960")
    if headless:
        loadPrcFileData("", "window-type offscreen")


def square_inverse_moi(inverse_mass: float, radius: float) -> float:
    if inverse_mass <= 0.0:
        return 0.0
    return inverse_mass / ((2.0 / 3.0) * radius * radius)


def world_tangent(theta: float) -> Vec3:
    return Vec3(math.cos(theta), 0.0, -math.sin(theta))


def world_normal(theta: float) -> Vec3:
    return Vec3(math.sin(theta), 0.0, math.cos(theta))


def vector_to_list(v: Vec3) -> list[float]:
    return [float(v.x), float(v.y), float(v.z)]


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

    vertex_data = GeomVertexData("friction-ramp", GeomVertexFormat.getV3c4(), Geom.UHStatic)
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

    node = GeomNode("friction-ramp")
    node.addGeom(geom)
    return node


class FrictionScene(ShowBase):
    def __init__(self, config: FrictionConfig):
        super().__init__(windowType="offscreen" if config.headless else None)
        self.sim_config = config
        self.dt = config.dt
        self.t = 0.0
        self.iFrame = 0
        self.g = config.gravity
        self.eCoeffRestitution = config.restitution
        self.muFrictionStatic = config.mu_static
        self.muFrictionKinetic = config.mu_kinetic
        self.rest_frames = 0
        self.last_contact_count = 0
        self.static_contact_this_step = False
        self.sliding_contact_this_step = False
        self.history: list[dict] = []

        lens = OrthographicLens()
        lens.setFilmSize(10.0, 8.0)
        lens.setNearFar(-100.0, 100.0)
        self.cam.node().setLens(lens)
        self.cam.setPos(0.0, -25.0, 0.0)
        self.cam.lookAt(0.0, 0.0, 0.0)

        dlight = DirectionalLight("dlight")
        dlight.setColor((0.95, 0.95, 0.95, 1.0))
        dlnp = render.attachNewNode(dlight)
        dlnp.setPos(16.0, -20.0, 18.0)
        dlnp.lookAt(0.0, 0.0, 0.0)
        render.setLight(dlnp)

        self.cube = loader.loadModel("Cube")
        self.cube.setScale(0.6)

        self.RigidBody2Ds: list[RigidBody2D] = []
        self.dynamic_body: RigidBody2D | None = None
        self.plane_body: RigidBody2D | None = None
        self.visual_ramp = None
        self.incline_theta = math.radians(config.angle_deg)
        self.incline_tangent = world_tangent(self.incline_theta)
        self.incline_normal = world_normal(self.incline_theta)
        self.setup_incline_scene()

        if not config.headless:
            self.taskMgr.add(self.update_task, "updateRigidBody2Ds")

    def setup_body(self, body: RigidBody2D, radius: float, mass: float | None, color: Vec3) -> None:
        body.radius = radius
        body.shape = "square"
        body.collisionRadius = 1.8 * radius
        body.setScale(radius)
        if mass is None or mass <= 0.0:
            body.inverseMass = 0.0
            body.inverseMomentOfInertia = 0.0
        else:
            body.inverseMass = 1.0 / mass
            body.inverseMomentOfInertia = square_inverse_moi(body.inverseMass, radius)
        body.setColor(color)

    def setup_incline_scene(self) -> None:
        for body in self.RigidBody2Ds:
            body.removeNode()

        self.RigidBody2Ds = []
        for index in range(2):
            body = RigidBody2D(f"body-{index}")
            if index == 0:
                self.cube.instanceTo(body)
            body.reparentTo(render)
            body.vel = Vec3(0.0, 0.0, 0.0)
            body.vold = Vec3(0.0, 0.0, 0.0)
            body.rold = Vec3(0.0, 0.0, 0.0)
            body.doGravity = False
            self.RigidBody2Ds.append(body)

        self.dynamic_body = self.RigidBody2Ds[0]
        self.plane_body = self.RigidBody2Ds[1]

        self.setup_body(self.dynamic_body, radius=1.0, mass=1.0, color=Vec3(0.85, 0.2, 0.15))
        self.dynamic_body.doGravity = True
        self.dynamic_body.sleeping = False
        self.dynamic_body.theta = self.incline_theta

        self.setup_body(self.plane_body, radius=4.0, mass=None, color=Vec3(0.35, 0.35, 0.42))
        self.plane_body.setPos(0.0, 3.0, -5.0)
        self.plane_body.theta = self.incline_theta

        if self.visual_ramp is not None:
            self.visual_ramp.removeNode()
        surface_origin = self.plane_body.getPos() + self.incline_normal * self.plane_body.radius
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

        tangent_offset = -2.0
        initial_gap = 0.025
        self.dynamic_body.setPos(
            surface_origin
            + self.incline_tangent * tangent_offset
            + self.incline_normal * (self.dynamic_body.radius + initial_gap)
        )
        self.dynamic_body.omega = 0.0
        self.rest_frames = 0

        self.apply_visual_transform(self.dynamic_body)

    def apply_visual_transform(self, body: RigidBody2D) -> None:
        body.setHpr(90.0, math.degrees(body.theta), 0.0)

    def update_task(self, task):
        self.step_physics()
        return task.cont

    def apply_impulse(self, body: RigidBody2D, impulse: Vec3, offset: Vec3, sign: float) -> None:
        if body.inverseMass <= 0.0:
            return
        body.vel += impulse * (sign * body.inverseMass)
        body.omega += (offset.cross(impulse)).y * sign * body.inverseMomentOfInertia

    def contact_tangent(self, normal: Vec3) -> Vec3:
        tangent = Vec3(normal.z, 0.0, -normal.x)
        if tangent.length() <= EPSILON:
            tangent = Vec3(self.incline_tangent)
        else:
            tangent.normalize()
        if tangent.dot(self.incline_tangent) < 0.0:
            tangent = -tangent
        return tangent

    def apply_tangential_position_correction(
        self,
        pi: RigidBody2D,
        pj: RigidBody2D,
        tangent: Vec3,
        inside_static_cone: bool,
    ) -> None:
        percent = (
            getattr(self.sim_config, "tangential_static_position_percent", 0.0)
            if inside_static_cone
            else getattr(self.sim_config, "tangential_sliding_position_percent", 0.0)
        )
        if percent <= 0.0:
            return

        total_inverse_mass = pi.inverseMass + pj.inverseMass
        if total_inverse_mass <= EPSILON:
            return

        drift_i = 0.0 if pi.inverseMass <= 0.0 else (pi.getPos() - pi.rold).dot(tangent)
        drift_j = 0.0 if pj.inverseMass <= 0.0 else (pj.getPos() - pj.rold).dot(tangent)
        relative_drift = drift_j - drift_i
        if abs(relative_drift) <= EPSILON:
            return

        correction = tangent * (percent * relative_drift)
        if pi.inverseMass > 0.0:
            pi.setPos(pi.getPos() + correction * (pi.inverseMass / total_inverse_mass))
        if pj.inverseMass > 0.0:
            pj.setPos(pj.getPos() - correction * (pj.inverseMass / total_inverse_mass))

    def snap_body_to_rest_surface(self) -> None:
        if self.dynamic_body is None or self.plane_body is None:
            return

        plane_surface_origin = self.plane_body.getPos() + self.incline_normal * self.plane_body.radius
        s = (self.dynamic_body.getPos() - plane_surface_origin).dot(self.incline_tangent)
        snapped_pos = plane_surface_origin + self.incline_tangent * s + self.incline_normal * self.dynamic_body.radius
        self.dynamic_body.setPos(snapped_pos)
        self.dynamic_body.theta = self.incline_theta
        self.dynamic_body.vel = Vec3(0.0, 0.0, 0.0)
        self.dynamic_body.omega = 0.0
        if getattr(self.sim_config, "disable_gravity_when_resting", False):
            self.dynamic_body.doGravity = False
            self.dynamic_body.sleeping = True

    def resolve_contact(self, pi: RigidBody2D, pj: RigidBody2D, depth: float, xclose: Vec3, normal: Vec3) -> bool:
        dxi = xclose - pi.getPos()
        dxj = xclose - pj.getPos()

        ui = pi.vel + Vec3(0.0, pi.omega, 0.0).cross(dxi)
        uj = pj.vel + Vec3(0.0, pj.omega, 0.0).cross(dxj)
        uij = uj - ui
        vn = uij.dot(normal)
        if vn >= 0.0:
            return False

        rni = dxi.cross(normal).y
        rnj = dxj.cross(normal).y
        k_normal = (
            pi.inverseMass
            + pj.inverseMass
            + (rni * rni) * pi.inverseMomentOfInertia
            + (rnj * rnj) * pj.inverseMomentOfInertia
        )
        if k_normal <= EPSILON:
            return False

        impulse_normal = normal * ((1.0 + self.eCoeffRestitution) * vn / k_normal)
        self.apply_impulse(pi, impulse_normal, dxi, +1.0)
        self.apply_impulse(pj, impulse_normal, dxj, -1.0)

        ui = pi.vel + Vec3(0.0, pi.omega, 0.0).cross(dxi)
        uj = pj.vel + Vec3(0.0, pj.omega, 0.0).cross(dxj)
        uij = uj - ui
        tangential_velocity = uij - normal * uij.dot(normal)
        tangential_speed = tangential_velocity.length()

        inside_static_cone = True
        tangent = self.contact_tangent(normal)
        if tangential_speed > 1.0e-7:
            tangent = tangential_velocity / tangential_speed
            rti = dxi.cross(tangent).y
            rtj = dxj.cross(tangent).y
            k_tangent = (
                pi.inverseMass
                + pj.inverseMass
                + (rti * rti) * pi.inverseMomentOfInertia
                + (rtj * rtj) * pj.inverseMomentOfInertia
            )
            if k_tangent > EPSILON:
                desired_tangent_mag = tangential_speed / k_tangent
                normal_mag = abs(impulse_normal.dot(normal))
                static_limit = self.muFrictionStatic * normal_mag
                inside_static_cone = desired_tangent_mag <= static_limit
                if inside_static_cone:
                    tangent_mag = desired_tangent_mag
                else:
                    tangent_mag = self.muFrictionKinetic * normal_mag
                impulse_tangent = tangent * tangent_mag
                self.apply_impulse(pi, impulse_tangent, dxi, +1.0)
                self.apply_impulse(pj, impulse_tangent, dxj, -1.0)
        self.static_contact_this_step = self.static_contact_this_step or inside_static_cone
        self.sliding_contact_this_step = self.sliding_contact_this_step or (not inside_static_cone)

        if depth > self.sim_config.position_slop:
            total_inverse_mass = pi.inverseMass + pj.inverseMass
            if total_inverse_mass > EPSILON:
                correction = self.sim_config.position_percent * (depth - self.sim_config.position_slop)
                if pi.inverseMass > 0.0:
                    pi.setPos(pi.getPos() - normal * (correction * pi.inverseMass / total_inverse_mass))
                if pj.inverseMass > 0.0:
                    pj.setPos(pj.getPos() + normal * (correction * pj.inverseMass / total_inverse_mass))
                self.apply_tangential_position_correction(pi, pj, tangent, inside_static_cone)

        return True

    def maybe_snap_to_rest(self) -> None:
        if not self.sim_config.enable_rest_snap or self.dynamic_body is None or self.plane_body is None:
            return

        if getattr(self.dynamic_body, "sleeping", False):
            self.dynamic_body.vel = Vec3(0.0, 0.0, 0.0)
            self.dynamic_body.omega = 0.0
            return

        tangential_speed = abs(self.dynamic_body.vel.dot(self.incline_tangent))
        normal_speed = abs(self.dynamic_body.vel.dot(self.incline_normal))
        if (
            self.last_contact_count >= 1
            and tangential_speed < self.sim_config.rest_speed_threshold
            and normal_speed < self.sim_config.rest_speed_threshold
            and abs(self.dynamic_body.omega) < self.sim_config.rest_omega_threshold
            and self.muFrictionStatic + 1.0e-4 >= math.tan(self.incline_theta)
        ):
            self.rest_frames += 1
        else:
            self.rest_frames = 0

        if self.rest_frames < self.sim_config.rest_contact_frames:
            return

        self.snap_body_to_rest_surface()

    def step_physics(self) -> None:
        markerClean(self)

        for body in self.RigidBody2Ds:
            if body.inverseMass <= 0.0:
                continue
            if getattr(body, "sleeping", False):
                body.vold = Vec3(body.vel)
                body.rold = Vec3(body.getPos())
                body.vel = Vec3(0.0, 0.0, 0.0)
                body.omega = 0.0
                continue
            acceleration = Vec3(0.0, 0.0, -self.g) if body.doGravity else Vec3(0.0, 0.0, 0.0)
            body.vold = Vec3(body.vel)
            body.rold = Vec3(body.getPos())
            body.vel += acceleration * self.dt
            body.setPos(body.getPos() + body.vel * self.dt)
            body.theta += body.omega * self.dt

        self.last_contact_count = 0
        self.static_contact_this_step = False
        self.sliding_contact_this_step = False
        for _ in range(self.sim_config.solver_iterations):
            for index, pi in enumerate(self.RigidBody2Ds):
                for pj in self.RigidBody2Ds[index + 1 :]:
                    xi = pi.getPos()
                    xj = pj.getPos()
                    if (xj - xi).length() >= pi.collisionRadius + pj.collisionRadius:
                        continue
                    contacts = collideShapes(self, pi, pj)
                    for depth, xclose, normal in contacts:
                        if xclose is None or normal is None:
                            continue
                        if self.resolve_contact(pi, pj, float(depth), xclose, normal.normalized()):
                            self.last_contact_count += 1

        self.maybe_snap_to_rest()
        self.t += self.dt
        self.iFrame += 1

        for body in self.RigidBody2Ds:
            self.apply_visual_transform(body)

        if self.sim_config.capture_history and self.dynamic_body is not None and self.plane_body is not None:
            plane_surface_origin = self.plane_body.getPos() + self.incline_normal * self.plane_body.radius
            rel = self.dynamic_body.getPos() - plane_surface_origin
            self.history.append(
                {
                    "time": self.t,
                    "position": vector_to_list(self.dynamic_body.getPos()),
                    "velocity": vector_to_list(self.dynamic_body.vel),
                    "theta": float(self.dynamic_body.theta),
                    "omega": float(self.dynamic_body.omega),
                    "tangent_position": float(rel.dot(self.incline_tangent)),
                    "normal_offset": float(rel.dot(self.incline_normal) - self.dynamic_body.radius),
                    "tangent_velocity": float(self.dynamic_body.vel.dot(self.incline_tangent)),
                    "normal_velocity": float(self.dynamic_body.vel.dot(self.incline_normal)),
                    "contact_count": int(self.last_contact_count),
                    "sleeping": bool(getattr(self.dynamic_body, "sleeping", False)),
                    "gravity_enabled": bool(self.dynamic_body.doGravity),
                }
            )

    def run_for_duration(self, duration: float) -> dict:
        nsteps = int(round(duration / self.dt))
        for _ in range(nsteps):
            self.step_physics()
            if self.sim_config.headless:
                self.graphicsEngine.renderFrame()

        if self.sim_config.screenshot_path:
            self.save_screenshot(self.sim_config.screenshot_path)

        summary = self.build_summary()
        if self.sim_config.report_path:
            self.write_report(self.sim_config.report_path, summary)
        return summary

    def save_screenshot(self, path: str) -> None:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.graphicsEngine.renderFrame()
        self.win.saveScreenshot(Filename.fromOsSpecific(path))

    def build_summary(self) -> dict:
        if not self.history:
            raise RuntimeError("No history was captured.")

        tangent_velocities = [entry["tangent_velocity"] for entry in self.history]
        times = [entry["time"] for entry in self.history]
        positions = [entry["tangent_position"] for entry in self.history]
        contact_fraction = sum(1 for entry in self.history if entry["contact_count"] > 0) / len(self.history)

        half_index = len(times) // 2
        if len(times) - half_index >= 2:
            dt_window = times[-1] - times[half_index]
            avg_accel = (tangent_velocities[-1] - tangent_velocities[half_index]) / max(dt_window, EPSILON)
        else:
            avg_accel = 0.0

        result = {
            "config": asdict(self.sim_config),
            "critical_mu": math.tan(self.incline_theta),
            "final_position": self.history[-1]["position"],
            "final_velocity": self.history[-1]["velocity"],
            "final_speed": float(Vec3(*self.history[-1]["velocity"]).length()),
            "final_tangent_position": positions[-1],
            "final_tangent_velocity": tangent_velocities[-1],
            "final_omega": self.history[-1]["omega"],
            "final_sleeping": self.history[-1]["sleeping"],
            "final_gravity_enabled": self.history[-1]["gravity_enabled"],
            "max_abs_tangent_velocity": max(abs(v) for v in tangent_velocities),
            "avg_tangent_accel_last_half": avg_accel,
            "contact_fraction": contact_fraction,
            "sleep_fraction": sum(1 for entry in self.history if entry["sleeping"]) / len(self.history),
            "total_tangent_displacement": positions[-1] - positions[0],
            "history": self.history,
        }
        return result

    def write_report(self, path: str, summary: dict) -> None:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="2D rigid body friction homework solution.")
    parser.add_argument("--headless", action="store_true", help="Run without an on-screen window.")
    parser.add_argument("--duration", type=float, default=6.0, help="Simulation duration in seconds.")
    parser.add_argument("--angle", type=float, default=13.0, help="Incline angle in degrees.")
    parser.add_argument("--mu", type=float, default=0.5, help="Static and kinetic friction coefficient.")
    parser.add_argument("--mu-static", type=float, default=None, help="Static friction coefficient.")
    parser.add_argument("--mu-kinetic", type=float, default=None, help="Kinetic friction coefficient.")
    parser.add_argument("--restitution", type=float, default=0.1, help="Coefficient of restitution.")
    parser.add_argument("--dt", type=float, default=1.0 / 120.0, help="Fixed time step.")
    parser.add_argument("--solver-iterations", type=int, default=1, help="Velocity solver passes per frame.")
    parser.add_argument("--screenshot", type=str, default=None, help="Optional PNG path for the final frame.")
    parser.add_argument("--report", type=str, default=None, help="Optional JSON path for the run report.")
    parser.add_argument(
        "--sweep",
        type=str,
        default=None,
        help="Optional JSON path. When set, a coefficient sweep is performed instead of a single run.",
    )
    parser.add_argument(
        "--snap-rest",
        action="store_true",
        help="Enable the resting-contact snap strategy used by the extra-credit variant.",
    )
    return parser.parse_args()


def make_config(args: argparse.Namespace) -> FrictionConfig:
    mu_static = args.mu if args.mu_static is None else args.mu_static
    mu_kinetic = args.mu if args.mu_kinetic is None else args.mu_kinetic
    return FrictionConfig(
        angle_deg=args.angle,
        dt=args.dt,
        restitution=args.restitution,
        mu_static=mu_static,
        mu_kinetic=mu_kinetic,
        duration=args.duration,
        headless=args.headless,
        screenshot_path=args.screenshot,
        report_path=args.report,
        solver_iterations=args.solver_iterations,
        enable_rest_snap=args.snap_rest,
    )


def run_scene(config: FrictionConfig) -> dict:
    configure_prc(config.headless)
    scene = FrictionScene(config)
    try:
        return scene.run_for_duration(config.duration)
    finally:
        scene.destroy()


def run_sweep(args: argparse.Namespace, output_path: str) -> None:
    sweep_values = [0.12, 0.18, 0.22, 0.24, 0.30, 0.50, 0.80]
    results = []
    for mu in sweep_values:
        run_args = argparse.Namespace(**vars(args))
        run_args.mu = mu
        run_args.mu_static = mu
        run_args.mu_kinetic = mu
        run_args.report = None
        run_args.screenshot = None
        run_args.headless = True
        summary = run_scene(make_config(run_args))
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
        config.report_path = None
        config.solver_iterations = max(config.solver_iterations, 4)
        config.position_percent = 1.0
        config.position_slop = min(config.position_slop, 5.0e-4)
        config.enable_rest_snap = False
        configure_prc(False)
        scene = FrictionScene(config)
        scene.run()
        return

    summary = run_scene(config)
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
