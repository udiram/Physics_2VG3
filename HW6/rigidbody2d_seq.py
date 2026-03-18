import argparse
import math
import os
import sys
from collections import deque
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent
VENV_PYTHON = ROOT_DIR / ".venv" / "Scripts" / "python.exe"

try:
    from panda3d.core import Filename, OrthographicLens, Vec3, loadPrcFileData
except ModuleNotFoundError as exc:
    # Allow the documented "python.exe rigidbody2d_seq.py ..." command to work
    # by relaunching under the project virtualenv when Panda3D is only installed there.
    if (
        exc.name == "panda3d.core"
        and VENV_PYTHON.exists()
        and Path(sys.executable).resolve() != VENV_PYTHON.resolve()
    ):
        os.execv(str(VENV_PYTHON), [str(VENV_PYTHON), __file__, *sys.argv[1:]])
    raise

# Local copies of HW5 modules are placed in the project root so imports are local.

from direct.showbase.ShowBase import ShowBase
from rigidbody2d_mod import RigidBody2D
from rigidbody2d_collision_mod import collideShapes, markerClean, markerInit


RAD_TO_DEG = 180.0 / math.pi
INCLINE_ANGLE = math.asin(0.1)
PLANE_POS = Vec3(0.2, 3.0, -3.0)
PALETTE = [
    Vec3(0.95, 0.45, 0.35),
    Vec3(0.20, 0.65, 0.95),
    Vec3(0.95, 0.75, 0.20),
    Vec3(0.55, 0.80, 0.30),
    Vec3(0.85, 0.40, 0.75),
    Vec3(0.80, 0.80, 0.80),
]


def panda_path(path):
    return Filename.fromOsSpecific(str(path)).getFullpath()


def parse_args():
    parser = argparse.ArgumentParser(description="2VG3 HW6 sequential impulse solver")
    parser.add_argument("--scenario", type=int, default=0, help="Scenario index (0-6)")
    parser.add_argument("--frames", type=int, default=0, help="Run this many frames then exit")
    parser.add_argument("--headless", action="store_true", help="Run without opening a window")
    parser.add_argument("--offscreen", action="store_true", help="Render offscreen")
    parser.add_argument("--report", action="store_true", help="Print a short simulation summary on exit")
    parser.add_argument(
        "--exercise2-table",
        action="store_true",
        help="Run the Exercise 2 iteration sweep for scenario 0 or 1 and print the table values",
    )
    parser.add_argument("--niter", type=int, default=None, help="Override solver iteration count")
    parser.add_argument("--bias-factor", type=float, default=None, help="Override velocity bias factor")
    parser.add_argument("--bias-slop", type=float, default=None, help="Override velocity bias slop")
    parser.add_argument("--screenshot", type=str, default="", help="Save a screenshot on exit")
    return parser.parse_args()


def configure_window(args):
    # Use Panda3D's "none" window type for pure simulation runs and
    # offscreen rendering when a screenshot is requested.
    if args.screenshot and not args.offscreen and not args.headless:
        args.offscreen = True
    if args.offscreen:
        loadPrcFileData("", "window-type offscreen")
    elif args.headless:
        loadPrcFileData("", "window-type none")
    loadPrcFileData("", "audio-library-name null")


def exercise2_frame_budget(scenario):
    # The report measurements were taken after these fixed run lengths.
    return 600 if scenario == 0 else 240


def exercise3_frame_budget(scenario):
    return 240 if scenario in (2, 3) else 0


class SimpleScene(ShowBase):
    def __init__(self, args):
        self.args = args
        super().__init__()

        if self.cam is not None:
            lens = OrthographicLens()
            lens.setFilmSize(8.0, 6.0)
            lens.setNearFar(-100, 100)
            self.cam.node().setLens(lens)

        self.g = 10.0
        self.dt = 1 / 60.0
        self.t = 0.0
        self.iFrame = 0
        self.eCoeffRestitution = 0.05
        self.muFrictionStatic = 0.5
        self.muFrictionKinetic = 0.5
        self.nIterations = 120
        self.biasFactor = 0.3
        self.biasSlop = 0.005
        self.fixPos = True
        self.positionIterations = 1
        self.positionPercent = 0.05
        self.positionSlop = 0.0005

        self.cube = loader.loadModel(panda_path(ROOT_DIR / "panda3d" / "Cube.egg"))
        # Cube.egg has half-extents of about 1.64042, not 1.0.
        # Normalize it so render scale matches the collision square radius.
        self.cube.setScale(1.0 / 1.6404199475065617)
        self.markerModel = panda_path(ROOT_DIR / "panda3d" / "sphere.egg.pz")
        markerInit(self, self.markerModel)

        self.scenario = args.scenario % 7
        self.RigidBody2Ds = []
        self.recentMaxSpeeds = deque(maxlen=120)
        self.reset_metrics()
        self.setScenario(self.scenario)

        self.accept("n", self.nextScenario)
        self.accept("p", self.prevScenario)
        self.accept("r", self.reloadScenario)

        self.gameTask = taskMgr.add(self.updateRigidBody2Ds, "updateRigidBody2Ds")

    def reset_metrics(self):
        self.initialEnergy = 0.0
        self.lastEnergy = 0.0
        self.minEnergy = 0.0
        self.maxEnergy = 0.0
        self.prevEnergy = 0.0
        self.preCollisionEnergy = None
        self.postCollisionEnergy = None
        self.preCollisionVelocity = None
        self.postCollisionVelocity = None
        self.prevPrimaryVelocity = None
        self.firstContactFrame = None
        self.peakContactCount = 0
        self.totalContactFrames = 0
        self.recentMaxSpeeds.clear()

    def nextScenario(self):
        self.setScenario(self.scenario + 1)

    def prevScenario(self):
        self.setScenario(self.scenario - 1)

    def reloadScenario(self):
        self.setScenario(self.scenario)

    def setScenario(self, scenario):
        self.scenario = scenario % 7
        self.t = 0.0
        self.iFrame = 0
        self.reset_metrics()

        for body in self.RigidBody2Ds:
            body.removeNode()

        # Rebuild the scene from scratch so switching scenarios always resets
        # bodies, solver parameters, and diagnostic counters together.
        self.configure_scenario()
        self.apply_cli_overrides()
        self.initialEnergy = self.total_energy()
        self.lastEnergy = self.initialEnergy
        self.minEnergy = self.initialEnergy
        self.maxEnergy = self.initialEnergy
        self.prevEnergy = self.initialEnergy
        primary = self.primary_body()
        if primary is not None:
            self.prevPrimaryVelocity = Vec3(primary.vel)
        print(
            "scenario=%i e=%.3f mu=%.3f niter=%i bias=%.3f"
            % (
                self.scenario,
                self.eCoeffRestitution,
                self.muFrictionStatic,
                self.nIterations,
                self.biasFactor,
            )
        )

    def apply_cli_overrides(self):
        if self.args.niter is not None:
            self.nIterations = self.args.niter
        if self.args.bias_factor is not None:
            self.biasFactor = self.args.bias_factor
        if self.args.bias_slop is not None:
            self.biasSlop = self.args.bias_slop

    def configure_scenario(self):
        self.RigidBody2Ds = []
        body_count = 6 if self.scenario in (5, 6) else 2
        for i in range(body_count):
            body = RigidBody2D(f"body{i}")
            body.reparentTo(render)
            body.shape = "square"
            body.setColor(PALETTE[i % len(PALETTE)])
            self.cube.instanceTo(body)
            self.RigidBody2Ds.append(body)

        self.fixPos = True
        self.dt = 1 / 60.0
        self.biasFactor = 0.3
        self.biasSlop = 0.005
        self.nIterations = 120
        self.positionIterations = 1
        self.positionPercent = 0.05
        self.positionSlop = 0.0005

        # Scenarios 0-4 are the two-body tests from Exercises 2-4.
        # Scenarios 5-6 are the stack and moving-frame stack from Exercise 5.
        if self.scenario == 0:
            self.eCoeffRestitution = 0.05
            self.muFrictionStatic = 0.5
            self.muFrictionKinetic = 0.5
            self.setup_block_plane(
                angle=INCLINE_ANGLE,
                block_vel=Vec3(-math.sin(INCLINE_ANGLE), 0.0, -math.cos(INCLINE_ANGLE)),
                gravity_on=True,
            )
        elif self.scenario == 1:
            self.eCoeffRestitution = 0.0
            self.muFrictionStatic = 0.5
            self.muFrictionKinetic = 0.5
            self.setup_block_plane(angle=0.0, block_vel=Vec3(0.0, 0.0, -1.0), gravity_on=False)
        elif self.scenario == 2:
            self.eCoeffRestitution = 1.0
            self.muFrictionStatic = 0.0
            self.muFrictionKinetic = 0.0
            self.setup_block_plane(
                angle=INCLINE_ANGLE,
                block_vel=Vec3(-math.sin(INCLINE_ANGLE), 0.0, -math.cos(INCLINE_ANGLE)),
                gravity_on=False,
            )
        elif self.scenario == 3:
            self.eCoeffRestitution = 1.0
            self.muFrictionStatic = 0.0
            self.muFrictionKinetic = 0.0
            self.setup_block_plane(angle=0.0, block_vel=Vec3(0.0, 0.0, -1.0), gravity_on=False)
        elif self.scenario == 4:
            self.eCoeffRestitution = 1.0
            self.muFrictionStatic = 0.0
            self.muFrictionKinetic = 0.0
            self.biasFactor = 0.005
            self.biasSlop = 0.015
            self.setup_block_plane(angle=0.0, block_vel=Vec3(0.0, 0.0, -1.0), gravity_on=True)
        elif self.scenario == 5:
            self.eCoeffRestitution = 0.05
            self.muFrictionStatic = 0.2
            self.muFrictionKinetic = 0.2
            self.nIterations = 400
            self.biasFactor = 0.05
            self.biasSlop = 0.01
            self.positionIterations = 4
            self.positionPercent = 0.1
            self.positionSlop = 0.001
            self.setup_stack(frame_velocity=Vec3(0.0, 0.0, 0.0))
        else:
            self.eCoeffRestitution = 0.05
            self.muFrictionStatic = 0.2
            self.muFrictionKinetic = 0.2
            self.nIterations = 400
            self.biasFactor = 0.05
            self.biasSlop = 0.01
            self.positionIterations = 4
            self.positionPercent = 0.1
            self.positionSlop = 0.001
            self.setup_stack(frame_velocity=Vec3(-1.0, 0.0, 0.0))

    def setup_block_plane(self, angle, block_vel, gravity_on):
        # Body 0 is the moving test block, body 1 is the static plane.
        block = self.RigidBody2Ds[0]
        plane = self.RigidBody2Ds[1]

        self.configure_box(
            block,
            radius=0.5,
            inverse_mass=1.0,
            pos=Vec3(0.0, 3.0, 0.5),
            vel=block_vel,
            theta=angle,
            gravity_on=gravity_on,
        )
        self.configure_box(
            plane,
            radius=2.0,
            inverse_mass=0.0,
            pos=Vec3(PLANE_POS),
            vel=Vec3(0.0, 0.0, 0.0),
            theta=angle,
            gravity_on=False,
        )

    def setup_stack(self, frame_velocity):
        # Keep the lecture's slightly offset stack so the solver has to handle
        # genuine multi-contact balancing, not a perfectly centered tower.
        fall_velocity = Vec3(-math.sin(INCLINE_ANGLE), 0.0, -math.cos(INCLINE_ANGLE))

        self.configure_box(
            self.RigidBody2Ds[0],
            radius=0.5,
            inverse_mass=1.0,
            pos=Vec3(0.0, 3.0, 0.5),
            vel=fall_velocity + frame_velocity,
            theta=INCLINE_ANGLE,
            gravity_on=True,
        )
        self.configure_box(
            self.RigidBody2Ds[1],
            radius=2.0,
            inverse_mass=0.0,
            pos=Vec3(PLANE_POS),
            vel=Vec3(frame_velocity),
            theta=INCLINE_ANGLE,
            gravity_on=False,
        )

        for i, body in enumerate(self.RigidBody2Ds[2:], start=2):
            radius = 0.3 - i * 0.01
            x_offset = -0.1 + (i % 3) * 0.05
            z_pos = 2.0 + (i - 2) * 0.5
            self.configure_box(
                body,
                radius=radius,
                inverse_mass=1.0 / 0.2,
                pos=Vec3(x_offset, 3.0, z_pos),
                vel=fall_velocity + frame_velocity,
                theta=INCLINE_ANGLE,
                gravity_on=True,
            )

    def configure_box(self, body, radius, inverse_mass, pos, vel, theta, gravity_on):
        # The physics uses "radius" as half-width for squares, so the render
        # scale and the collision geometry should both use the same value.
        body.radius = radius
        body.collisionRadius = 1.8 * radius
        body.inverseMass = inverse_mass
        body.inverseMomentOfInertia = self.square_inverse_moi(inverse_mass, radius)
        body.vel = Vec3(vel)
        body.doGravity = gravity_on
        body.theta = theta
        body.omega = 0.0
        body.setScale(radius)
        body.setPos(pos)
        body.setHpr(90.0, theta * RAD_TO_DEG, 0.0)

    @staticmethod
    def square_inverse_moi(inverse_mass, radius):
        if inverse_mass == 0.0:
            return 0.0
        return inverse_mass / ((2.0 / 3.0) * radius * radius)

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

    def primary_body(self):
        for body in self.RigidBody2Ds:
            if body.inverseMass != 0.0:
                return body
        return None

    def compute_velocity_bias(self, dclose, dt):
        # Bias helps push out penetrating contacts, but too much of it adds
        # energy. Later scenarios therefore tune the bias separately.
        if dclose < self.biasSlop:
            return 0.0
        return dclose * self.biasFactor / dt

    def collect_contacts(self):
        # Build a contact list from the provisional positions for this frame.
        # The stored xi/xj/xclose snapshot is reused throughout the sequential
        # impulse iterations so each constraint stays tied to one contact.
        constraints = []
        for i, body_i in enumerate(self.RigidBody2Ds):
            for body_j in self.RigidBody2Ds[i + 1 :]:
                xi = Vec3(body_i.getPos())
                xj = Vec3(body_j.getPos())
                if (xj - xi).length() >= body_i.collisionRadius + body_j.collisionRadius:
                    continue
                for dclose, xclose, normal in collideShapes(self, body_i, body_j):
                    if xclose is None or normal.lengthSquared() == 0.0:
                        continue
                    constraints.append(
                        {
                            "pi": body_i,
                            "pj": body_j,
                            "xi": xi,
                            "xj": xj,
                            "xclose": Vec3(xclose),
                            "normal": Vec3(normal.normalized()),
                            "dclose": float(dclose),
                            "dp_normal": 0.0,
                            "dp_fric": Vec3(0.0, 0.0, 0.0),
                            "target_normal": 0.0,
                        }
                    )
        return constraints

    def relative_contact_velocity(self, constraint):
        pi = constraint["pi"]
        pj = constraint["pj"]
        xi = constraint["xi"]
        xj = constraint["xj"]
        xclose = constraint["xclose"]
        ui = pi.vel + Vec3(0.0, pi.omega, 0.0).cross(xclose - xi)
        uj = pj.vel + Vec3(0.0, pj.omega, 0.0).cross(xclose - xj)
        return uj - ui

    def solve_contacts(self, constraints, dt):
        # Record the desired post-collision normal velocity once, then iteratively
        # apply impulse deltas while clamping the accumulated impulse totals.
        for constraint in constraints:
            uij = self.relative_contact_velocity(constraint)
            unormal = uij.dot(constraint["normal"])
            vbias = self.compute_velocity_bias(constraint["dclose"], dt)
            constraint["target_normal"] = -self.eCoeffRestitution * unormal + vbias

        for _ in range(self.nIterations):
            for constraint in constraints:
                pi = constraint["pi"]
                pj = constraint["pj"]
                xi = constraint["xi"]
                xj = constraint["xj"]
                xclose = constraint["xclose"]
                nij = constraint["normal"]
                dxi = xclose - xi
                dxj = xclose - xj

                uij = self.relative_contact_velocity(constraint)
                unormal = uij.dot(nij)

                rni = dxi.cross(nij)
                rnj = dxj.cross(nij)
                inv_mass_normal = (
                    pi.inverseMass
                    + pj.inverseMass
                    + rni.dot(rni) * pi.inverseMomentOfInertia
                    + rnj.dot(rnj) * pj.inverseMomentOfInertia
                )
                if inv_mass_normal == 0.0:
                    continue

                # Sequential impulses work with an incremental impulse update,
                # not a one-shot solve. The total normal impulse is clamped so
                # it can only separate the bodies, never pull them together.
                dp_normal_delta = (unormal - constraint["target_normal"]) / inv_mass_normal
                dp_normal_new = constraint["dp_normal"] + dp_normal_delta
                if dp_normal_new > 0.0:
                    dp_normal_new = 0.0
                dp_normal_delta = dp_normal_new - constraint["dp_normal"]
                constraint["dp_normal"] = dp_normal_new

                fric_vec = uij - nij * unormal
                if fric_vec.lengthSquared() > 1e-20:
                    fric_dir = Vec3(fric_vec.normalized())
                    rnfi = dxi.cross(fric_dir)
                    rnfj = dxj.cross(fric_dir)
                    inv_mass_fric = (
                        pi.inverseMass
                        + pj.inverseMass
                        + rnfi.dot(rnfi) * pi.inverseMomentOfInertia
                        + rnfj.dot(rnfj) * pj.inverseMomentOfInertia
                    )
                    if inv_mass_fric != 0.0:
                        dp_fric_delta = fric_vec / inv_mass_fric
                    else:
                        dp_fric_delta = Vec3(0.0, 0.0, 0.0)
                else:
                    dp_fric_delta = Vec3(0.0, 0.0, 0.0)

                # Friction uses the same accumulated-impulse idea, but the
                # total tangential impulse is limited by the friction cone.
                dp_fric_new = constraint["dp_fric"] + dp_fric_delta
                fric_ratio = dp_fric_new.length() / (abs(dp_normal_new) + 1e-20)
                if fric_ratio > self.muFrictionStatic and dp_fric_new.lengthSquared() > 0.0:
                    dp_fric_new *= self.muFrictionKinetic / fric_ratio
                    dp_fric_delta = dp_fric_new - constraint["dp_fric"]
                constraint["dp_fric"] = dp_fric_new

                delta_impulse = nij * dp_normal_delta + dp_fric_delta
                pi.vel += delta_impulse * pi.inverseMass
                pj.vel -= delta_impulse * pj.inverseMass
                pi.omega += dxi.cross(delta_impulse).y * pi.inverseMomentOfInertia
                pj.omega -= dxj.cross(delta_impulse).y * pj.inverseMomentOfInertia

    def project_positions(self):
        if not self.fixPos:
            return

        # Velocity correction alone can leave small visible overlap, especially
        # in stacks. This pass gently projects bodies apart along the current
        # contact normals without changing the physical setup.
        for _ in range(self.positionIterations):
            had_overlap = False
            max_penetration = 0.0

            for constraint in self.collect_contacts():
                dclose = constraint["dclose"]
                if dclose <= self.positionSlop:
                    continue

                pi = constraint["pi"]
                pj = constraint["pj"]
                inv_mass_sum = pi.inverseMass + pj.inverseMass
                if inv_mass_sum == 0.0:
                    continue

                had_overlap = True
                max_penetration = max(max_penetration, dclose)

                correction_mag = self.positionPercent * (dclose - self.positionSlop) / inv_mass_sum
                correction = constraint["normal"] * correction_mag

                pi.setPos(pi.getPos() - correction * pi.inverseMass)
                pj.setPos(pj.getPos() + correction * pj.inverseMass)

            if not had_overlap or max_penetration <= self.positionSlop:
                break

    def update_metrics(self, contact_count):
        primary = self.primary_body()
        current_primary_velocity = Vec3(primary.vel) if primary is not None else None

        if contact_count > 0:
            self.totalContactFrames += 1
            self.peakContactCount = max(self.peakContactCount, contact_count)
            if self.firstContactFrame is None:
                self.firstContactFrame = self.iFrame
                self.preCollisionEnergy = self.prevEnergy
                if self.prevPrimaryVelocity is not None:
                    self.preCollisionVelocity = Vec3(self.prevPrimaryVelocity)
        elif self.firstContactFrame is not None and self.postCollisionEnergy is None:
            self.postCollisionEnergy = self.lastEnergy
            if current_primary_velocity is not None:
                self.postCollisionVelocity = Vec3(current_primary_velocity)

        max_speed = 0.0
        for body in self.RigidBody2Ds:
            if body.inverseMass != 0.0:
                max_speed = max(max_speed, body.vel.length())
        self.recentMaxSpeeds.append(max_speed)
        if current_primary_velocity is not None:
            self.prevPrimaryVelocity = Vec3(current_primary_velocity)

    def updateRigidBody2Ds(self, task):
        markerClean(self)

        dt = self.dt
        self.t += dt
        self.iFrame += 1

        # First do an unconstrained velocity/position update. These provisional
        # poses are only used to detect contacts for the solver.
        for body in self.RigidBody2Ds:
            accel = Vec3(0.0, 0.0, 0.0)
            if body.doGravity:
                accel = Vec3(0.0, 0.0, -self.g)
            body.vold = Vec3(body.vel)
            body.rold = Vec3(body.getPos())
            body.thetaold = body.theta
            body.vel += accel * dt
            body.setPos(body.rold + body.vel * dt)
            body.theta += body.omega * dt
            body.setHpr(90.0, body.theta * RAD_TO_DEG, 0.0)

        constraints = self.collect_contacts()
        if constraints:
            self.solve_contacts(constraints, dt)
            # Throw away the provisional positions and rebuild them from the
            # corrected velocities, matching the lecture's sequential-impulse flow.
            for body in self.RigidBody2Ds:
                body.setPos(body.rold + (body.vold + body.vel) * 0.5 * dt)
                body.theta = body.thetaold + body.omega * dt
                body.setHpr(90.0, body.theta * RAD_TO_DEG, 0.0)
            self.project_positions()

        self.prevEnergy = self.lastEnergy
        self.lastEnergy = self.total_energy()
        self.minEnergy = min(self.minEnergy, self.lastEnergy)
        self.maxEnergy = max(self.maxEnergy, self.lastEnergy)
        self.update_metrics(len(constraints))

        return task.cont

    def print_summary(self):
        recent_max_speed = max(self.recentMaxSpeeds) if self.recentMaxSpeeds else 0.0
        print(
            "SUMMARY scenario=%i frames=%i contact_frames=%i peak_contacts=%i "
            "energy_initial=%.6f energy_final=%.6f energy_min=%.6f energy_max=%.6f "
            "recent_max_speed=%.6f"
            % (
                self.scenario,
                self.iFrame,
                self.totalContactFrames,
                self.peakContactCount,
                self.initialEnergy,
                self.lastEnergy,
                self.minEnergy,
                self.maxEnergy,
                recent_max_speed,
            )
        )
        if self.preCollisionEnergy is not None:
            after_energy = self.postCollisionEnergy
            if after_energy is None:
                after_energy = self.lastEnergy
            print(
                "COLLISION_ENERGY before=%.6f after=%.6f"
                % (self.preCollisionEnergy, after_energy)
            )
            if self.scenario in (2, 3):
                delta_energy = after_energy - self.preCollisionEnergy
                rel_error = abs(delta_energy) / (abs(self.preCollisionEnergy) + 1e-20)
                passed = abs(delta_energy) < 1e-6
                print(
                    "EX3_ENERGY scenario=%i before=%.6f after=%.6f delta=%.6e rel_error=%.6e pass=%s"
                    % (
                        self.scenario,
                        self.preCollisionEnergy,
                        after_energy,
                        delta_energy,
                        rel_error,
                        passed,
                    )
                )
        for i, body in enumerate(self.RigidBody2Ds):
            if body.inverseMass == 0.0:
                continue
            pos = body.getPos()
            vel = body.vel
            print(
                "BODY %i pos=(%.6f, %.6f, %.6f) vel=(%.6f, %.6f, %.6f) omega=%.6f"
                % (i, pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, body.omega)
            )
        if self.scenario in (0, 1):
            body = self.RigidBody2Ds[0]
            pos = body.getPos()
            print(
                "EX2_METRIC scenario=%i niter=%i speed=%.6f omega=%.6f pos=(%.6f, %.6f, %.6f)"
                % (
                    self.scenario,
                    self.nIterations,
                    body.vel.length(),
                    body.omega,
                    pos.x,
                    pos.y,
                    pos.z,
                )
            )
        if self.scenario in (2, 3) and self.preCollisionVelocity is not None:
            outgoing = self.postCollisionVelocity
            if outgoing is None:
                primary = self.primary_body()
                if primary is not None:
                    outgoing = Vec3(primary.vel)
            target = -self.preCollisionVelocity
            print(
                "EX3_METRIC scenario=%i incoming=(%.6f, %.6f, %.6f) outgoing=(%.6f, %.6f, %.6f) "
                "target=(%.6f, %.6f, %.6f)"
                % (
                    self.scenario,
                    self.preCollisionVelocity.x,
                    self.preCollisionVelocity.y,
                    self.preCollisionVelocity.z,
                    outgoing.x,
                    outgoing.y,
                    outgoing.z,
                    target.x,
                    target.y,
                    target.z,
                )
            )
        if self.scenario == 4 and self.preCollisionEnergy is not None:
            after_energy = self.postCollisionEnergy
            if after_energy is None:
                after_energy = self.lastEnergy
            print(
                "EX4_METRIC bias=%.6f bias_slop=%.6f initial_total=%.6f final_total=%.6f delta=%.6f"
                % (
                    self.biasFactor,
                    self.biasSlop,
                    self.initialEnergy,
                    self.lastEnergy,
                    self.lastEnergy - self.initialEnergy,
                )
            )
        if self.scenario in (5, 6):
            dynamic_bodies = [body for body in self.RigidBody2Ds if body.inverseMass != 0.0]
            speeds = [body.vel.length() for body in dynamic_bodies]
            x_velocities = [body.vel.x for body in dynamic_bodies]
            label = "EX5_METRIC" if self.scenario == 5 else "EX6_METRIC"
            print(
                "%s niter=%i bias=%.6f bias_slop=%.6f min_speed=%.6f max_speed=%.6f min_vx=%.6f max_vx=%.6f"
                % (
                    label,
                    self.nIterations,
                    self.biasFactor,
                    self.biasSlop,
                    min(speeds),
                    max(speeds),
                    min(x_velocities),
                    max(x_velocities),
                )
            )


def run_headless(scene, args):
    frames = args.frames if args.frames > 0 else 600
    for _ in range(frames):
        scene.taskMgr.step()
    if args.screenshot and scene.win is not None:
        scene.graphicsEngine.renderFrame()
        scene.graphicsEngine.renderFrame()
        filename = Filename.fromOsSpecific(str(Path(args.screenshot).resolve()))
        scene.win.saveScreenshot(filename)
    if args.report:
        scene.print_summary()
    scene.destroy()


def run_exercise2_table(args):
    if args.scenario not in (0, 1):
        print("Exercise 2 table mode only applies to scenario 0 or 1.")
        return

    # Reproduce the exact iteration sweep used to fill the Exercise 2 table.
    frames = exercise2_frame_budget(args.scenario)
    iteration_values = [1, 2, 5, 10, 20, 50, 120]

    print(
        "EX2_TABLE_START scenario=%i frames=%i iterations=%s"
        % (args.scenario, frames, iteration_values)
    )
    print("Columns: niter, speed, omega, pos_x, pos_z")

    for niter in iteration_values:
        study_args = argparse.Namespace(**vars(args))
        study_args.frames = 0
        study_args.report = False
        study_args.screenshot = ""
        scene = SimpleScene(study_args)
        scene.nIterations = niter
        for _ in range(frames):
            scene.taskMgr.step()

        body = scene.RigidBody2Ds[0]
        pos = body.getPos()
        print(
            "EX2_TABLE_ROW niter=%i speed=%.6f omega=%.6f pos_x=%.6f pos_z=%.6f"
            % (niter, body.vel.length(), body.omega, pos.x, pos.z)
        )
        scene.destroy()

    print("EX2_TABLE_END")


def main():
    args = parse_args()
    if args.exercise2_table:
        args.headless = True
        args.offscreen = False
    configure_window(args)
    if args.exercise2_table:
        run_exercise2_table(args)
        return
    scene = SimpleScene(args)
    if args.headless or args.offscreen or args.frames > 0 or args.screenshot:
        run_headless(scene, args)
    else:
        scene.run()


if __name__ == "__main__":
    main()
