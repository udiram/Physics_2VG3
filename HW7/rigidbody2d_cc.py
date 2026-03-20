import argparse
import copy
import math
import os
import sys
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent
VENV_PYTHON = ROOT_DIR / ".venv" / "Scripts" / "python.exe"

try:
    from panda3d.core import Filename, OrthographicLens, Vec3, loadPrcFileData
except ModuleNotFoundError as exc:
    if (
        exc.name == "panda3d.core"
        and VENV_PYTHON.exists()
        and Path(sys.executable).resolve() != VENV_PYTHON.resolve()
    ):
        os.execv(str(VENV_PYTHON), [str(VENV_PYTHON), __file__, *sys.argv[1:]])
    raise

from direct.showbase.ShowBase import ShowBase
from rigidbody2d_constraints_mod import constraintContactList
from rigidbody2d_collide_mod import collideContactList
from rigidbody2d_markers_mod import markerClean, markerInit
from rigidbody2d_mod import RigidBody2D


RAD_TO_DEG = 180.0 / math.pi
DEFAULT_SCENARIO = 3
Y_LEVEL = 3.0
GRAVITY = 10.0
PENDULUM_LENGTH = 2.0
PENDULUM_RADIUS = 0.25
PENDULUM_MASS = 1.0
CRADLE_RADIUS = 0.25
STACK_INCLINE_ANGLE = math.asin(0.1)
STACK_PLANE_POS = Vec3(0.2, Y_LEVEL, -3.0)
BODY_COLORS = [
    Vec3(0.86, 0.24, 0.20),
    Vec3(0.12, 0.52, 0.82),
    Vec3(0.90, 0.66, 0.14),
    Vec3(0.18, 0.66, 0.35),
    Vec3(0.66, 0.32, 0.70),
    Vec3(0.28, 0.28, 0.28),
]


def panda_path(path):
    return Filename.fromOsSpecific(str(path)).getFullpath()


def parse_args():
    parser = argparse.ArgumentParser(description="2VG3 HW7 constraints and collisions")
    parser.add_argument("--scenario", type=int, default=DEFAULT_SCENARIO, help="Scenario index (0-4)")
    parser.add_argument("--frames", type=int, default=0, help="Run this many frames then exit")
    parser.add_argument("--headless", action="store_true", help="Run without opening a window")
    parser.add_argument("--offscreen", action="store_true", help="Render offscreen")
    parser.add_argument("--report", action="store_true", help="Print a short report and exit")
    parser.add_argument(
        "--pendulum-period-mode",
        action="store_true",
        help="Force scenario 2 and print an explicit pendulum period readout in the terminal",
    )
    parser.add_argument(
        "--cradle-readout-mode",
        action="store_true",
        help="Force scenario 3 and print a simple Newton's cradle readout in the terminal",
    )
    parser.add_argument(
        "--stack-readout-mode",
        action="store_true",
        help="Force scenario 4 and print a simple stack readout in the terminal",
    )
    parser.add_argument("--niter", type=int, default=None, help="Override the solver iteration count")
    parser.add_argument(
        "--pendulum-vx",
        type=float,
        default=-2.0,
        help="Initial x velocity for the pendulum bob or leftmost cradle bob",
    )
    parser.add_argument("--nsphere", type=int, default=3, help="Number of spheres in Newton's cradle")
    parser.add_argument(
        "--bias-mode",
        choices=["catto", "none", "speed-gated", "persistent"],
        default="persistent",
        help="Collision velocity-bias strategy",
    )
    parser.add_argument(
        "--collision-bias-factor",
        type=float,
        default=None,
        help="Override the collision bias coefficient",
    )
    parser.add_argument(
        "--collision-bias-slop",
        type=float,
        default=None,
        help="Override the collision penetration slop",
    )
    parser.add_argument(
        "--dynamic-speed-cutoff",
        type=float,
        default=0.5,
        help="For speed-gated bias, turn off bias above this closing speed",
    )
    parser.add_argument(
        "--persistent-contact-frames",
        type=int,
        default=4,
        help="For persistent bias, require this many consecutive contact frames before bias turns on",
    )
    parser.add_argument(
        "--constraint-bias-factor",
        type=float,
        default=None,
        help="Override the hinge bias coefficient",
    )
    return parser.parse_args()


def configure_window(args):
    if args.offscreen:
        loadPrcFileData("", "window-type offscreen")
    elif args.headless or args.frames > 0 or args.report:
        loadPrcFileData("", "window-type none")
    loadPrcFileData("", "audio-library-name null")


def default_frame_budget(scenario):
    if scenario == 2:
        return 1800
    if scenario == 3:
        return 1200
    if scenario == 4:
        return 1500
    return 600


class SimpleScene(ShowBase):
    def __init__(self, args):
        self.args = args
        self.nSphere = max(2, args.nsphere)
        super().__init__()

        if self.cam is not None:
            lens = OrthographicLens()
            lens.setFilmSize(8.0, 6.0)
            lens.setNearFar(-100, 100)
            self.cam.node().setLens(lens)

        self.g = GRAVITY
        self.dt = 1 / 60.0
        self.t = 0.0
        self.iFrame = 0
        self.fixPos = True
        self.doFriction = True
        self.eCoeffRestitution = 1.0
        self.muFrictionStatic = 0.5
        self.muFrictionKinetic = 0.5
        self.nIterations = 100

        self.biasMode = args.bias_mode
        self.collisionBiasFactor = 0.3
        self.collisionBiasSlop = 0.01
        self.dynamicSpeedCutoff = args.dynamic_speed_cutoff
        self.constraintBiasFactor = 0.05

        self.cube = loader.loadModel(panda_path(ROOT_DIR / "Cube.egg"))
        self.cube.setScale(1.0 / 1.6404199475065617)
        self.tube = loader.loadModel(panda_path(ROOT_DIR / "Tube.egg"))
        self.tube.setScale(1.0 / 1.121591329574585)
        self.tube.setHpr(90.0, 90.0, 0.0)
        self.tube.setPos(0.0, 0.1, 0.0)

        markerInit(self, panda_path(ROOT_DIR / "panda3d" / "sphere.egg.pz"))

        self.RigidBody2Ds = []
        self.constraintList = []
        self.scenario = 0
        self.reset_metrics()
        self.set_scenario(args.scenario)

        self.accept("n", self.next_scenario)
        self.accept("p", self.previous_scenario)
        self.accept("r", self.reload_scenario)
        self.gameTask = taskMgr.add(self.updateRigidBody2Ds, "updateRigidBody2Ds")

    def reset_metrics(self):
        self.initialEnergy = 0.0
        self.lastEnergy = 0.0
        self.energyMin = 0.0
        self.energyMax = 0.0
        self.maxCollisionDepth = 0.0
        self.lastCollisionDepth = 0.0
        self.zeroCrossings = []
        self.periodEstimates = []
        self.lastZeroCrossTime = None
        self.maxPendulumAbsX = 0.0
        self.cradlePeakAbsVX = []
        self.cradleMultiActiveFrames = 0
        self.cradleMiddleMotionFrames = 0
        self.activityThreshold = 0.10
        self.contactAgeByPair = {}

    def next_scenario(self):
        self.set_scenario(self.scenario + 1)

    def previous_scenario(self):
        self.set_scenario(self.scenario - 1)

    def reload_scenario(self):
        self.set_scenario(self.scenario)

    def set_scenario(self, scenario):
        for body in self.RigidBody2Ds:
            body.removeNode()

        self.RigidBody2Ds = []
        self.constraintList = []
        self.scenario = scenario % 5
        self.t = 0.0
        self.iFrame = 0
        self.reset_metrics()
        self.setup_scenario()
        self.apply_cli_overrides()
        self.initialEnergy = self.total_energy()
        self.lastEnergy = self.initialEnergy
        self.energyMin = self.initialEnergy
        self.energyMax = self.initialEnergy
        if not (self.args.pendulum_period_mode or self.args.cradle_readout_mode or self.args.stack_readout_mode):
            print(
                "scenario=%i e=%.3f niter=%i bias_mode=%s"
                % (self.scenario, self.eCoeffRestitution, self.nIterations, self.biasMode)
            )

    def apply_cli_overrides(self):
        if self.args.niter is not None:
            self.nIterations = self.args.niter
        if self.args.collision_bias_factor is not None:
            self.collisionBiasFactor = self.args.collision_bias_factor
        if self.args.collision_bias_slop is not None:
            self.collisionBiasSlop = self.args.collision_bias_slop
        if self.args.constraint_bias_factor is not None:
            self.constraintBiasFactor = self.args.constraint_bias_factor

    def setup_scenario(self):
        self.dt = 1 / 60.0
        self.doFriction = True
        self.eCoeffRestitution = 1.0
        self.muFrictionStatic = 0.5
        self.muFrictionKinetic = 0.5
        self.nIterations = 100
        self.collisionBiasFactor = 0.3
        self.collisionBiasSlop = 0.01
        self.constraintBiasFactor = 0.05

        if self.scenario == 0:
            self.setup_demo_hinge()
        elif self.scenario == 1:
            self.setup_demo_car()
        elif self.scenario == 2:
            self.setup_pendulum()
        elif self.scenario == 3:
            self.setup_newtons_cradle()
        else:
            self.setup_stack()

    def add_block(self, name, radius, pos, vel, inverse_mass, theta=0.0, do_gravity=False, color=None):
        body = RigidBody2D("block")
        body.name = name
        body.shape = "square"
        body.radius = radius
        body.collisionRadius = 1.8 * radius
        body.inverseMass = inverse_mass
        body.inverseMomentOfInertia = self.square_inverse_moi(inverse_mass, radius)
        body.theta = theta
        body.omega = 0.0
        body.vel = Vec3(vel)
        body.doGravity = do_gravity
        body.noCollideList = []
        body.setPos(pos)
        body.setScale(radius)
        body.setHpr(90.0, theta * RAD_TO_DEG, 0.0)
        if color is not None:
            body.setColor(color)
        body.reparentTo(render)
        self.cube.instanceTo(body)
        self.RigidBody2Ds.append(body)
        return body

    def add_cylinder(
        self,
        name,
        radius,
        pos,
        vel,
        mass,
        theta=0.0,
        do_gravity=False,
        color=None,
        no_collide=None,
    ):
        body = RigidBody2D("cylinder")
        body.name = name
        body.shape = "cylinder"
        body.radius = radius
        body.collisionRadius = 1.1 * radius
        body.inverseMass = 0.0 if mass == 0.0 else 1.0 / mass
        body.inverseMomentOfInertia = self.cylinder_inverse_moi(body.inverseMass, radius)
        body.theta = theta
        body.omega = 0.0
        body.vel = Vec3(vel)
        body.doGravity = do_gravity
        body.noCollideList = list(no_collide or [])
        body.setPos(pos)
        body.setScale(radius)
        body.setHpr(90.0, theta * RAD_TO_DEG, 0.0)
        if color is not None:
            body.setColor(color)
        body.reparentTo(render)
        self.tube.instanceTo(body)
        self.RigidBody2Ds.append(body)
        return body

    def body_by_name(self, name):
        for body in self.RigidBody2Ds:
            if body.name == name:
                return body
        raise KeyError(name)

    @staticmethod
    def square_inverse_moi(inverse_mass, radius):
        if inverse_mass == 0.0:
            return 0.0
        return inverse_mass / ((2.0 / 3.0) * radius * radius)

    @staticmethod
    def cylinder_inverse_moi(inverse_mass, radius):
        if inverse_mass == 0.0:
            return 0.0
        return inverse_mass / (radius * radius)

    def setup_demo_hinge(self):
        angle = 0.0
        self.eCoeffRestitution = 1.0

        self.add_block(
            name="plane",
            radius=2.0,
            pos=Vec3(0.2, Y_LEVEL, -3.0),
            vel=Vec3(0.0, 0.0, 0.0),
            inverse_mass=0.0,
            theta=0.0,
            do_gravity=False,
            color=Vec3(1.0, 0.0, 0.0),
        )

        block1 = self.add_block(
            name="block1",
            radius=0.5,
            pos=Vec3(0.0, Y_LEVEL, 0.5),
            vel=Vec3(-math.sin(angle), 0.0, -math.cos(angle)),
            inverse_mass=1.0,
            theta=0.0,
            do_gravity=True,
            color=Vec3(0.15, 0.55, 0.85),
        )
        block2 = self.add_block(
            name="block2",
            radius=0.3,
            pos=Vec3(-0.7, Y_LEVEL, 1.5),
            vel=Vec3(-math.sin(angle), 0.0, -math.cos(angle)),
            inverse_mass=1.0 / 0.5,
            theta=-math.pi * 0.4,
            do_gravity=True,
            color=Vec3(0.85, 0.45, 0.12),
        )
        self.constraintList.append(
            ("hinge", block1, block2, Vec3(-1.0, 0.0, 1.0) * block1.radius * 1.01, Vec3(-1.0, 0.0, -1.0) * block2.radius * 1.01)
        )

    def setup_demo_car(self):
        self.eCoeffRestitution = 0.1
        angle = 0.10016742116
        plane_radius = 20.0

        self.add_block(
            name="plane",
            radius=plane_radius,
            pos=Vec3(0.0, Y_LEVEL, -plane_radius - 1.5),
            vel=Vec3(0.0, 0.0, 0.0),
            inverse_mass=0.0,
            theta=angle,
            do_gravity=False,
            color=Vec3(0.85, 0.2, 0.2),
        )
        car = self.add_block(
            name="car",
            radius=1.0,
            pos=Vec3(-1.0, Y_LEVEL, 1.0),
            vel=Vec3(0.0, 0.0, 0.0),
            inverse_mass=1.0,
            theta=0.0,
            do_gravity=True,
            color=Vec3(0.0, 0.7, 0.7),
        )
        wheel1 = self.add_cylinder(
            name="wheel1",
            radius=1.0,
            pos=Vec3(-2.0, Y_LEVEL, 0.0),
            vel=Vec3(0.0, 0.0, 0.0),
            mass=1.0,
            do_gravity=True,
            color=Vec3(0.25, 0.25, 0.25),
            no_collide=["car"],
        )
        wheel2 = self.add_cylinder(
            name="wheel2",
            radius=0.8,
            pos=Vec3(0.0, Y_LEVEL, 0.0),
            vel=Vec3(0.0, 0.0, 0.0),
            mass=1.0,
            do_gravity=True,
            color=Vec3(0.15, 0.15, 0.15),
            no_collide=["car"],
        )
        self.constraintList.append(("hinge", car, wheel1, Vec3(-1.0, 0.0, -1.0) * car.radius, Vec3(0.0, 0.0, 0.0)))
        self.constraintList.append(("hinge", car, wheel2, Vec3(1.0, 0.0, -1.0) * car.radius, Vec3(0.0, 0.0, 0.0)))

    def setup_pendulum(self):
        self.eCoeffRestitution = 1.0
        self.doFriction = False
        self.nIterations = 120

        anchor = self.add_block(
            name="anchor",
            radius=0.5,
            pos=Vec3(0.0, Y_LEVEL, 2.5),
            vel=Vec3(0.0, 0.0, 0.0),
            inverse_mass=0.0,
            theta=0.0,
            do_gravity=False,
            color=Vec3(0.75, 0.35, 0.25),
        )
        bob = self.add_cylinder(
            name="pendulum_bob",
            radius=PENDULUM_RADIUS,
            pos=Vec3(0.0, Y_LEVEL, 0.0),
            vel=Vec3(self.args.pendulum_vx, 0.0, 0.0),
            mass=PENDULUM_MASS,
            theta=0.0,
            do_gravity=True,
            color=Vec3(0.10, 0.45, 0.85),
        )
        self.constraintList.append(("hinge", anchor, bob, Vec3(0.0, 0.0, -anchor.radius), Vec3(0.0, 0.0, PENDULUM_LENGTH)))

    def setup_newtons_cradle(self):
        self.eCoeffRestitution = 1.0
        self.doFriction = False
        self.nIterations = 140

        beam_radius = max(1.0, 0.5 * self.nSphere)
        beam = self.add_block(
            name="beam",
            radius=beam_radius,
            pos=Vec3(0.0, Y_LEVEL, 2.0 + beam_radius),
            vel=Vec3(0.0, 0.0, 0.0),
            inverse_mass=0.0,
            theta=0.0,
            do_gravity=False,
            color=Vec3(0.28, 0.28, 0.28),
        )

        spacing = 2.0 * CRADLE_RADIUS
        x_center = 0.5 * (self.nSphere - 1) * spacing
        self.cradlePeakAbsVX = [0.0 for _ in range(self.nSphere)]
        for i in range(self.nSphere):
            x_pos = i * spacing - x_center
            x_vel = self.args.pendulum_vx if i == 0 else 0.0
            bob = self.add_cylinder(
                name=f"sphere{i}",
                radius=CRADLE_RADIUS,
                pos=Vec3(x_pos, Y_LEVEL, 0.0),
                vel=Vec3(x_vel, 0.0, 0.0),
                mass=1.0,
                theta=0.0,
                do_gravity=True,
                color=BODY_COLORS[i % len(BODY_COLORS)],
            )
            self.constraintList.append(("hinge", beam, bob, Vec3(x_pos, 0.0, -beam.radius), Vec3(0.0, 0.0, PENDULUM_LENGTH)))

    def setup_stack(self):
        self.eCoeffRestitution = 0.05
        self.muFrictionStatic = 0.2
        self.muFrictionKinetic = 0.2
        self.doFriction = True
        self.nIterations = 220
        self.collisionBiasFactor = 0.3
        self.collisionBiasSlop = 0.01

        self.add_block(
            name="plane",
            radius=2.0,
            pos=Vec3(STACK_PLANE_POS),
            vel=Vec3(0.0, 0.0, 0.0),
            inverse_mass=0.0,
            theta=STACK_INCLINE_ANGLE,
            do_gravity=False,
            color=Vec3(0.85, 0.2, 0.2),
        )

        fall_velocity = Vec3(-math.sin(STACK_INCLINE_ANGLE), 0.0, -math.cos(STACK_INCLINE_ANGLE))
        offsets = [0.00, -0.10, -0.05, 0.00, -0.10]
        for i, x_offset in enumerate(offsets):
            radius = 0.30 - i * 0.01
            z_pos = 0.55 + i * 0.52
            self.add_block(
                name=f"stack{i}",
                radius=radius,
                pos=Vec3(x_offset, Y_LEVEL, z_pos),
                vel=Vec3(fall_velocity),
                inverse_mass=1.0 / 0.2,
                theta=STACK_INCLINE_ANGLE,
                do_gravity=True,
                color=BODY_COLORS[i % len(BODY_COLORS)],
            )

    def total_energy(self):
        total = 0.0
        for body in self.RigidBody2Ds:
            if body.inverseMass == 0.0:
                continue
            mass = 1.0 / body.inverseMass
            total += 0.5 * mass * body.vel.lengthSquared()
            if body.inverseMomentOfInertia != 0.0:
                total += 0.5 * (body.omega * body.omega) / body.inverseMomentOfInertia
            if body.doGravity:
                total += mass * self.g * body.getPos().z
        return total

    def pair_key(self, pi, pj):
        return tuple(sorted((pi.name, pj.name)))

    def refresh_contact_ages(self, collision_contacts):
        next_ages = {}
        for contact in collision_contacts:
            _, _, pi, pj, *_ = contact
            key = self.pair_key(pi, pj)
            next_ages[key] = self.contactAgeByPair.get(key, 0) + 1
        self.contactAgeByPair = next_ages

    def collision_bias(self, dclose, dt, unormal, pi, pj):
        if self.biasMode == "none":
            return 0.0
        if dclose < self.collisionBiasSlop:
            return 0.0
        if self.biasMode == "persistent":
            closing_speed = max(0.0, -unormal)
            if closing_speed > self.dynamicSpeedCutoff:
                return 0.0
            if self.contactAgeByPair.get(self.pair_key(pi, pj), 0) < self.args.persistent_contact_frames:
                return 0.0
            return dclose * self.collisionBiasFactor / dt
        if self.biasMode == "speed-gated":
            closing_speed = max(0.0, -unormal)
            if closing_speed >= self.dynamicSpeedCutoff:
                return 0.0
            scale = 1.0 - closing_speed / max(self.dynamicSpeedCutoff, 1e-9)
            return dclose * self.collisionBiasFactor * scale / dt
        return dclose * self.collisionBiasFactor / dt

    def hinge_bias(self, drij, dt):
        return drij * self.constraintBiasFactor / dt

    def update_rigid_body(self, body, dt):
        accel = Vec3(0.0, 0.0, -self.g) if body.doGravity else Vec3(0.0, 0.0, 0.0)
        body.vold = Vec3(body.vel)
        body.rold = Vec3(body.getPos())
        body.thetaold = body.theta
        body.omegaold = body.omega
        body.vel = body.vel + accel * dt
        body.setPos(body.rold + body.vel * dt)
        body.theta += body.omega * dt
        body.setHpr(90.0, body.theta * RAD_TO_DEG, 0.0)

    def track_pendulum_metrics(self, bob, dt):
        x_old = bob.rold.x
        x_new = bob.getPos().x
        self.maxPendulumAbsX = max(self.maxPendulumAbsX, abs(x_new))
        if x_old == x_new or x_old * x_new > 0.0:
            return

        denom = abs(x_old) + abs(x_new)
        frac = 0.5 if denom == 0.0 else abs(x_old) / denom
        cross_time = self.t - dt + frac * dt
        if cross_time <= 0.25:
            return
        if self.lastZeroCrossTime is not None and cross_time - self.lastZeroCrossTime <= 0.5:
            return

        self.zeroCrossings.append(cross_time)
        if self.lastZeroCrossTime is not None:
            self.periodEstimates.append(2.0 * (cross_time - self.lastZeroCrossTime))
        self.lastZeroCrossTime = cross_time

    def track_cradle_metrics(self):
        active_count = 0
        middle_is_active = False
        for i in range(self.nSphere):
            bob = self.body_by_name(f"sphere{i}")
            abs_vx = abs(bob.vel.x)
            self.cradlePeakAbsVX[i] = max(self.cradlePeakAbsVX[i], abs_vx)
            if abs_vx > self.activityThreshold:
                active_count += 1
                if i not in (0, self.nSphere - 1):
                    middle_is_active = True
        if active_count > 1:
            self.cradleMultiActiveFrames += 1
        if middle_is_active:
            self.cradleMiddleMotionFrames += 1

    def track_stack_metrics(self, collision_contacts):
        if collision_contacts:
            current_depth = max(contact[4] for contact in collision_contacts)
            self.lastCollisionDepth = current_depth
            self.maxCollisionDepth = max(self.maxCollisionDepth, current_depth)

    def updateRigidBody2Ds(self, task):
        markerClean(self)

        dt = self.dt
        self.t += dt
        self.iFrame += 1

        for body in self.RigidBody2Ds:
            self.update_rigid_body(body, dt)

        constraint_contacts = constraintContactList(self)
        collision_contacts = collideContactList(self)
        self.refresh_contact_ages(collision_contacts)

        for _ in range(self.nIterations):
            for con in constraint_contacts:
                dp, _, pi, pj, xci, xcj, xi, xj = con
                drij = xcj - xci
                dxi = xci - xi
                dxj = xcj - xj
                ui = pi.vel + Vec3(0.0, pi.omega, 0.0).cross(dxi)
                uj = pj.vel + Vec3(0.0, pj.omega, 0.0).cross(dxj)
                uij = (uj - ui) + self.hinge_bias(drij, dt)
                if uij.lengthSquared() == 0.0:
                    continue
                nij = uij.normalized()
                rni = dxi.cross(nij)
                rnj = dxj.cross(nij)
                inv_mass = (
                    pi.inverseMass
                    + pj.inverseMass
                    + rni.dot(rni) * pi.inverseMomentOfInertia
                    + rnj.dot(rnj) * pj.inverseMomentOfInertia
                )
                if inv_mass == 0.0:
                    continue
                dp_delta = uij / inv_mass
                con[0] = dp + dp_delta

                pi.vel += dp_delta * pi.inverseMass
                pj.vel -= dp_delta * pj.inverseMass
                pi.omega += dxi.cross(dp_delta).y * pi.inverseMomentOfInertia
                pj.omega -= dxj.cross(dp_delta).y * pj.inverseMomentOfInertia

            for contact in collision_contacts:
                dp_scalar, dp_fric, pi, pj, dclose, _, nij, _, vnormal, dxi, dxj, iMFac = contact
                ui = pi.vel + Vec3(0.0, pi.omega, 0.0).cross(dxi)
                uj = pj.vel + Vec3(0.0, pj.omega, 0.0).cross(dxj)
                uij = uj - ui
                unormal = uij.dot(nij)
                vbias = self.collision_bias(dclose, dt, unormal, pi, pj)
                delta_dp_scalar = (unormal - (vnormal + vbias)) * iMFac
                dp_scalar_new = dp_scalar + delta_dp_scalar
                if dp_scalar_new > 0.0:
                    dp_scalar_new = 0.0
                delta_dp_scalar = dp_scalar_new - dp_scalar
                contact[0] = dp_scalar_new

                delta_dp_fric = Vec3(0.0, 0.0, 0.0)
                if self.doFriction:
                    fric_vec = uij - nij * unormal
                    if fric_vec.lengthSquared() > 1e-20:
                        fric_dir = fric_vec.normalized()
                        rni = dxi.cross(fric_dir)
                        rnj = dxj.cross(fric_dir)
                        inv_mass_fric = (
                            pi.inverseMass
                            + pj.inverseMass
                            + rni.dot(rni) * pi.inverseMomentOfInertia
                            + rnj.dot(rnj) * pj.inverseMomentOfInertia
                        )
                        if inv_mass_fric != 0.0:
                            delta_dp_fric = fric_vec / inv_mass_fric
                            dp_fric_new = dp_fric + delta_dp_fric
                            fric_ratio = dp_fric_new.length() / (abs(dp_scalar_new) + 1e-20)
                            if fric_ratio > self.muFrictionStatic and dp_fric_new.lengthSquared() > 0.0:
                                dp_fric_new *= self.muFrictionKinetic / fric_ratio
                            delta_dp_fric = dp_fric_new - dp_fric
                            contact[1] = dp_fric_new

                delta_impulse = nij * delta_dp_scalar + delta_dp_fric
                pi.vel += delta_impulse * pi.inverseMass
                pj.vel -= delta_impulse * pj.inverseMass
                pi.omega += dxi.cross(delta_impulse).y * pi.inverseMomentOfInertia
                pj.omega -= dxj.cross(delta_impulse).y * pj.inverseMomentOfInertia

        if constraint_contacts or collision_contacts:
            if self.fixPos:
                for body in self.RigidBody2Ds:
                    body.setPos(body.rold + (body.vel + body.vold) * 0.5 * dt)
                    body.theta = body.thetaold + (body.omega + body.omegaold) * 0.5 * dt
                    body.setHpr(90.0, body.theta * RAD_TO_DEG, 0.0)

        if self.scenario == 2:
            self.track_pendulum_metrics(self.body_by_name("pendulum_bob"), dt)
        elif self.scenario == 3:
            self.track_cradle_metrics()
        elif self.scenario == 4:
            self.track_stack_metrics(collision_contacts)

        self.lastEnergy = self.total_energy()
        self.energyMin = min(self.energyMin, self.lastEnergy)
        self.energyMax = max(self.energyMax, self.lastEnergy)
        return task.cont

    def print_summary(self):
        print(
            "SUMMARY scenario=%i frames=%i energy_initial=%.6f energy_final=%.6f energy_min=%.6f energy_max=%.6f"
            % (
                self.scenario,
                self.iFrame,
                self.initialEnergy,
                self.lastEnergy,
                self.energyMin,
                self.energyMax,
            )
        )

        if self.scenario == 2:
            ideal_period = 2.0 * math.pi * math.sqrt(PENDULUM_LENGTH / self.g)
            measured_period = 0.0
            if self.periodEstimates:
                measured_period = sum(self.periodEstimates[-4:]) / min(4, len(self.periodEstimates))
            period_text = ",".join("%.6f" % period for period in self.periodEstimates)
            print(
                "PENDULUM_PERIOD vx=%.3f ideal=%.6f measured=%.6f crossings=%i max_abs_x=%.6f periods=[%s]"
                % (
                    self.args.pendulum_vx,
                    ideal_period,
                    measured_period,
                    len(self.zeroCrossings),
                    self.maxPendulumAbsX,
                    period_text,
                )
            )
        elif self.scenario == 3:
            _, _, final_vx = self.cradle_stats()
            final_text = ",".join("%.6f" % value for value in final_vx)
            peak_text = ",".join("%.6f" % value for value in self.cradlePeakAbsVX)
            frame_count = max(1, self.iFrame)
            print(
                "CRADLE_METRIC nsphere=%i bias_mode=%s initial_vx=%.3f peak_vx=[%s] final_vx=[%s] multi_active_ratio=%.6f middle_motion_ratio=%.6f"
                % (
                    self.nSphere,
                    self.biasMode,
                    self.args.pendulum_vx,
                    peak_text,
                    final_text,
                    self.cradleMultiActiveFrames / frame_count,
                    self.cradleMiddleMotionFrames / frame_count,
                )
            )
        elif self.scenario == 4:
            _, final_depth, _, speeds = self.stack_stats()
            speed_text = ",".join("%.6f" % speed for speed in speeds)
            print(
                "STACK_METRIC bias_mode=%s max_depth_seen=%.6f final_depth=%.6f max_speed=%.6f speeds=[%s]"
                % (
                    self.biasMode,
                    self.maxCollisionDepth,
                    final_depth,
                    max(speeds) if speeds else 0.0,
                    speed_text,
                )
            )

    def pendulum_period_stats(self):
        ideal_period = 2.0 * math.pi * math.sqrt(PENDULUM_LENGTH / self.g)
        measured_period = 0.0
        sample_count = min(4, len(self.periodEstimates))
        if sample_count > 0:
            measured_period = sum(self.periodEstimates[-sample_count:]) / sample_count
        return ideal_period, measured_period, sample_count

    def cradle_stats(self):
        final_vx = []
        for i in range(self.nSphere):
            final_vx.append(self.body_by_name(f"sphere{i}").vel.x)
        return self.initialEnergy, self.lastEnergy, final_vx

    def stack_stats(self):
        final_contacts = collideContactList(self)
        final_depth = 0.0
        if final_contacts:
            final_depth = max(contact[4] for contact in final_contacts)
        dynamic_bodies = [body for body in self.RigidBody2Ds if body.inverseMass != 0.0]
        speeds = [body.vel.length() for body in dynamic_bodies]
        max_speed = max(speeds) if speeds else 0.0
        return self.maxCollisionDepth, final_depth, max_speed, speeds

    def print_pendulum_period_readout(self):
        if self.scenario != 2:
            print("Ideal period: not available")
            print("Calculated period: not available")
            print("Percent error: not available")
            return

        ideal_period, measured_period, sample_count = self.pendulum_period_stats()
        if sample_count > 0:
            percent_error = abs(measured_period - ideal_period) / ideal_period * 100.0
            measured_text = "%.6f" % measured_period
            error_text = "%.2f%%" % percent_error
        else:
            measured_text = "not enough data"
            error_text = "not available"
        print("Ideal period: %.6f" % ideal_period)
        print("Calculated period: %s" % measured_text)
        print("Percent error: %s" % error_text)

    def print_cradle_readout(self):
        if self.scenario != 3:
            print("Initial energy: not available")
            print("Final energy: not available")
            print("Final x velocities: not available")
            return

        initial_energy, final_energy, final_vx = self.cradle_stats()
        print("Initial energy: %.6f" % initial_energy)
        print("Final energy: %.6f" % final_energy)
        print("Final x velocities: %s" % ", ".join("%.6f" % value for value in final_vx))

    def print_stack_readout(self):
        if self.scenario != 4:
            print("Maximum penetration: not available")
            print("Final penetration: not available")
            print("Maximum box speed: not available")
            return

        max_depth_seen, final_depth, max_speed, _ = self.stack_stats()
        print("Maximum penetration: %.6f" % max_depth_seen)
        print("Final penetration: %.6f" % final_depth)
        print("Maximum box speed: %.6f" % max_speed)


def run_headless(scene, args):
    frames = args.frames if args.frames > 0 else default_frame_budget(scene.scenario)
    for _ in range(frames):
        scene.taskMgr.step()
    if args.report:
        scene.print_summary()
    if args.pendulum_period_mode:
        scene.print_pendulum_period_readout()
    if args.cradle_readout_mode:
        scene.print_cradle_readout()
    if args.stack_readout_mode:
        scene.print_stack_readout()
    scene.destroy()


def main():
    args = parse_args()
    if args.pendulum_period_mode:
        args.scenario = 2
        args.headless = True
        if args.frames == 0:
            args.frames = 2400
    if args.cradle_readout_mode:
        args.scenario = 3
        args.headless = True
        if args.frames == 0:
            args.frames = 720
    if args.stack_readout_mode:
        args.scenario = 4
        args.headless = True
        if args.frames == 0:
            args.frames = 1500
    configure_window(args)
    scene = SimpleScene(args)
    if args.headless or args.offscreen or args.frames > 0 or args.report:
        run_headless(scene, args)
    else:
        scene.run()


if __name__ == "__main__":
    main()
