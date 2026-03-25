import argparse
import copy
import json
import math
import sys
from math import cos, sin

from panda3d.core import *

if "--headless" in sys.argv:
    loadPrcFileData("", "window-type none")
elif "--offscreen" in sys.argv:
    loadPrcFileData("", "window-type offscreen")
loadPrcFileData("", "audio-library-name null")

from direct.showbase.ShowBase import ShowBase

from rigidbody2d_collide_mod import *
from rigidbody2d_constraints_mod import *
from rigidbody2d_markers_mod import *
from rigidbody2d_mod import *


DEFAULT_SCENARIO = 3
DEFAULT_NSPHERE = 3
DEFAULT_BIAS_MODE = "adaptive"
DEFAULT_CRADLE_GAP = 0.001


class SimpleScene(ShowBase):
    def __init__(self, args):
        super().__init__(self)

        lens = OrthographicLens()
        lens.setFilmSize(8.0, 6.0)
        lens.setNearFar(-100, 100)
        if self.cam is not None:
            self.cam.node().setLens(lens)

        self.g = 10.0
        self.eCoeffRestitution = 1.0
        self.muFrictionStatic = 0.5
        self.muFrictionKinetic = 0.5

        self.dt = 1 / 60.0
        self.t = 0.0
        self.iFrame = 0
        self.iColl = 0
        self.iMarker = 0
        self.fixPos = True
        self.fixVel = False
        self.doFriction = True

        self.scenario = args.scenario
        self.nSphere = args.nsphere
        self.biasMode = args.bias_mode
        self.constraintBiasFactor = args.constraint_bias
        self.collisionBiasFactor = args.collision_bias
        self.biasSpeedThreshold = args.bias_speed_threshold
        self.pendulumInitialVx = args.pendulum_vx
        self.cradleGap = args.cradle_gap
        self.headless = args.headless
        self.offscreen = args.offscreen
        self.screenshotPath = args.screenshot
        self.summaryJsonPath = args.summary_json
        self.historyJsonPath = args.history_json
        self.historyStride = max(1, args.history_stride)

        self.cube = loader.loadModel("Cube")
        self.cube.setScale(0.6)
        self.tube = loader.loadModel("Tube")
        self.tube.setScale(1.3 / 1.41421356237)
        self.tube.setHpr(90.0, 90.0, 0.0)
        self.tube.setPos(0, 0.1, 0)
        markerInit(self, "Cube")

        self.RigidBody2Ds = []
        self.constraintList = []
        self.supportBody = None
        self.pendulumBody = None
        self.pendulumCrossingTimes = []
        self.pendulumLastX = None
        self.pendulumLastT = None
        self.cradleBodies = []
        self.cradleRestX = []
        self.history = []

        self.nextScenario()
        self.gameTask = taskMgr.add(self.updateRigidBody2Ds, "updateRigidBody2Ds")

    def bodyByName(self, name):
        return [p for p in self.RigidBody2Ds if p.name == name][0]

    def add_body(
        self,
        name,
        model,
        pos,
        radius,
        shape="square",
        color=None,
        vel=None,
        gravity=False,
        inverse_mass=1.0,
        inverse_moi=None,
        theta=0.0,
        no_collide=None,
    ):
        body = RigidBody2D(name)
        body.name = name
        body.radius = radius
        body.shape = shape
        body.setScale(radius)
        body.setPos(pos)
        body.vel = Vec3(0, 0, 0) if vel is None else Vec3(vel)
        body.doGravity = gravity
        body.inverseMass = inverse_mass
        if shape == "cylinder":
            body.collisionRadius = 1.1 * radius
            default_inverse_moi = 0.0 if inverse_mass == 0.0 else inverse_mass / (radius**2)
        else:
            body.collisionRadius = 1.8 * radius
            default_inverse_moi = 0.0 if inverse_mass == 0.0 else inverse_mass / (2.0 / 3.0 * radius**2)
        body.inverseMomentOfInertia = default_inverse_moi if inverse_moi is None else inverse_moi
        body.theta = theta
        body.noCollideList = [] if no_collide is None else list(no_collide)
        if color is not None:
            body.setColor(Vec3(*color))
        model.instanceTo(body)
        body.reparentTo(render)
        self.RigidBody2Ds.append(body)
        return body

    def add_support(self, half_width, center_z, name="support"):
        self.supportBody = self.add_body(
            name=name,
            model=self.cube,
            pos=Vec3(0, 3, center_z),
            radius=half_width,
            color=(0.8, 0.2, 0.2),
            inverse_mass=0.0,
            inverse_moi=0.0,
        )
        return self.supportBody

    def add_pendulum(self, name, x_center, radius=0.25, z_center=0.0, vx=0.0):
        body = self.add_body(
            name=name,
            model=self.tube,
            pos=Vec3(x_center, 3, z_center),
            radius=radius,
            shape="cylinder",
            color=(0.2, 0.7, 0.9),
            vel=Vec3(vx, 0, 0),
            gravity=True,
            no_collide=["support"],
        )
        anchor_world = Vec3(x_center, 3, 2.0)
        constraintAdd(
            self,
            "hinge",
            self.supportBody,
            body,
            anchor_world - self.supportBody.getPos(),
            anchor_world - body.getPos(),
        )
        return body

    def reset_diagnostics(self):
        self.pendulumBody = None
        self.pendulumCrossingTimes = []
        self.pendulumLastX = None
        self.pendulumLastT = None
        self.cradleBodies = []
        self.cradleRestX = []
        self.history = []

    def nextScenario(self):
        self.dt = 1 / 60.0
        self.eCoeffRestitution = 1.0
        self.muFrictionStatic = 0.5
        self.muFrictionKinetic = 0.5
        self.reset_diagnostics()

        for p in self.RigidBody2Ds:
            p.removeNode()

        self.RigidBody2Ds = []
        self.constraintList = []
        self.supportBody = None
        angle = 0.0

        if self.scenario == 0:
            p = self.add_body(
                name="plane",
                model=self.cube,
                pos=Vec3(0.2, 3, -3.0),
                radius=2.0,
                color=(1, 0, 0),
                inverse_mass=0.0,
                inverse_moi=0.0,
            )

            p = self.add_body(
                name="block1",
                model=self.cube,
                pos=Vec3(0.0, 3, 0.5),
                radius=0.5,
                vel=Vec3(-sin(angle), 0, -cos(angle)),
                gravity=True,
            )

            p = self.add_body(
                name="block2",
                model=self.cube,
                pos=Vec3(-0.7, 3, 1.5),
                radius=0.3,
                vel=Vec3(-sin(angle), 0, -cos(angle)),
                gravity=True,
                inverse_mass=1 / 0.5,
                theta=-math.pi * 0.4,
            )

            pi = self.bodyByName("block1")
            pj = self.bodyByName("block2")
            self.constraintList.append(
                ("hinge", pi, pj, Vec3(-1, 0, 1) * pi.radius * 1.01, Vec3(-1, 0, -1) * pj.radius * 1.01)
            )

        elif self.scenario == 1:
            self.eCoeffRestitution = 0.1
            angle = 0.10016742116
            self.add_body(
                name="plane",
                model=self.cube,
                pos=Vec3(0, 3, -5.5),
                radius=4.0,
                inverse_mass=0.0,
                inverse_moi=0.0,
                theta=angle,
            )

            self.add_body(
                name="car",
                model=self.cube,
                pos=Vec3(-1, 3, 1),
                radius=1.0,
                color=(0, 0.7, 0.7),
                gravity=True,
            )

            self.add_body(
                name="wheel1",
                model=self.tube,
                pos=Vec3(-2, 3, 0),
                radius=1.0,
                shape="cylinder",
                gravity=True,
                no_collide=["car"],
            )

            self.add_body(
                name="wheel2",
                model=self.tube,
                pos=Vec3(0, 3, 0),
                radius=0.8,
                shape="cylinder",
                gravity=True,
                no_collide=["car"],
            )

            pi = self.bodyByName("car")
            pj = self.bodyByName("wheel1")
            self.constraintList.append(("hinge", pi, pj, Vec3(-1, 0, -1) * pi.radius, Vec3(0, 0, 0)))
            pj = self.bodyByName("wheel2")
            self.constraintList.append(("hinge", pi, pj, Vec3(1, 0, -1) * pi.radius, Vec3(0, 0, 0)))

        elif self.scenario == 2:
            self.add_support(0.5, 2.5)
            self.pendulumBody = self.add_pendulum("pendulum", 0.0, vx=self.pendulumInitialVx)

        elif self.scenario == 3:
            bob_radius = 0.25
            spacing = 2.0 * bob_radius + self.cradleGap
            support_half_width = max(1.5, 0.5 * self.nSphere * spacing + bob_radius)
            self.add_support(support_half_width, 3.5)
            x0 = -0.5 * (self.nSphere - 1) * spacing
            for i in range(self.nSphere):
                x = x0 + i * spacing
                vx = self.pendulumInitialVx if i == 0 else 0.0
                bob = self.add_pendulum(f"sphere{i}", x, radius=bob_radius, vx=vx)
                bob.setColor(Vec3(0.85, 0.85 - 0.1 * (i % 3), 0.25 + 0.2 * (i % 2)))
                self.cradleBodies.append(bob)
                self.cradleRestX.append(x)

        elif self.scenario == 4:
            self.eCoeffRestitution = 0.0
            self.muFrictionStatic = 0.6
            self.muFrictionKinetic = 0.5
            plane = self.add_body(
                name="plane",
                model=self.cube,
                pos=Vec3(0.0, 3, -5.0),
                radius=4.0,
                color=(0.8, 0.3, 0.2),
                inverse_mass=0.0,
                inverse_moi=0.0,
            )
            stack_radius = 0.35
            base_z = plane.getPos().z + plane.radius + stack_radius
            x_offsets = [0.0, 0.0, 0.01, -0.01]
            colors = [(0.25, 0.6, 0.9), (0.3, 0.75, 0.55), (0.95, 0.7, 0.25), (0.85, 0.35, 0.25)]
            for i, (xoff, color) in enumerate(zip(x_offsets, colors)):
                self.add_body(
                    name=f"stack{i}",
                    model=self.cube,
                    pos=Vec3(xoff, 3, base_z + i * 2.0 * stack_radius),
                    radius=stack_radius,
                    shape="square",
                    color=color,
                    gravity=True,
                )

        else:
            raise ValueError("Unsupported scenario %s" % self.scenario)

        print("scenario=%i e=%f bias=%s" % (self.scenario, self.eCoeffRestitution, self.biasMode))

        for p in self.RigidBody2Ds:
            p.setHpr(90.0, p.theta * 180 / math.pi, 0.0)

    def collision_vbias(self, dclose, unormal):
        if self.biasMode == "zero":
            return 0.0
        if dclose < 0.01:
            return 0.0
        base = dclose * self.collisionBiasFactor / self.dt
        if self.biasMode == "catto":
            return base
        speed_scale = max(0.0, 1.0 - min(abs(unormal) / self.biasSpeedThreshold, 1.0))
        return base * speed_scale

    def update_pendulum_period(self):
        if self.pendulumBody is None:
            return
        x = float(self.pendulumBody.getPos().x)
        if self.pendulumLastX is not None and self.pendulumLastT is not None:
            crossed = x == 0.0 or self.pendulumLastX == 0.0 or x * self.pendulumLastX < 0.0
            if crossed:
                dx = x - self.pendulumLastX
                crossing_time = self.t
                if dx != 0.0:
                    crossing_time = self.pendulumLastT - self.pendulumLastX * (self.t - self.pendulumLastT) / dx
                if not self.pendulumCrossingTimes or abs(crossing_time - self.pendulumCrossingTimes[-1]) > 0.5 * self.dt:
                    self.pendulumCrossingTimes.append(crossing_time)
        self.pendulumLastX = x
        self.pendulumLastT = self.t

    def average_pendulum_period(self):
        if len(self.pendulumCrossingTimes) < 2:
            return None
        periods = []
        for t0, t1 in zip(self.pendulumCrossingTimes[:-1], self.pendulumCrossingTimes[1:]):
            periods.append(2.0 * (t1 - t0))
        if not periods:
            return None
        return sum(periods) / len(periods)

    def moving_cradle_bodies(self, threshold=0.2):
        return [p.name for p in self.cradleBodies if math.hypot(p.vel.x, p.vel.z) > threshold]

    def body_snapshot(self, body):
        return {
            "name": body.name,
            "x": float(body.getPos().x),
            "z": float(body.getPos().z),
            "vx": float(body.vel.x),
            "vz": float(body.vel.z),
            "theta": float(body.theta),
            "omega": float(body.omega),
        }

    def capture_history_sample(self, force=False):
        if not self.historyJsonPath:
            return
        if not force and self.iFrame % self.historyStride != 0:
            return
        if self.history and abs(self.history[-1]["time"] - self.t) < 1.0e-9:
            return
        self.history.append({"time": float(self.t), "bodies": [self.body_snapshot(p) for p in self.RigidBody2Ds]})

    def updateRigidBody2Ds(self, task):
        markerClean(self)

        dt = self.dt
        self.t += self.dt
        self.iFrame += 1

        for p in self.RigidBody2Ds:
            a = Vec3(0.0, 0.0, 0.0)
            if p.doGravity:
                a = Vec3(0.0, 0.0, -self.g)
            p.vold = copy.deepcopy(p.vel)
            p.rold = p.getPos()
            p.thetaold = p.theta
            p.omegaold = p.omega

            p.vel = p.vel + a * dt
            p.setPos(p.rold + p.vel * dt)
            p.theta += p.omega * dt
            p.setHpr(90.0, p.theta * 180 / math.pi, 0.0)

        constraint_contacts = constraintContactList(self)
        collision_contacts = collideContactList(self)

        niter = 100
        for iter in range(niter):
            for con in constraint_contacts:
                dp, dummy, pi, pj, xci, xcj, xi, xj = con
                drij = xcj - xci
                dxi = xci - xi
                dxj = xcj - xj
                ui = pi.vel + Vec3(0.0, pi.omega, 0.0).cross(dxi)
                uj = pj.vel + Vec3(0.0, pj.omega, 0.0).cross(dxj)
                vbias = drij * self.constraintBiasFactor / dt
                uij = (uj - ui) + vbias
                nij = uij.normalized()
                rni = dxi.cross(nij)
                rnj = dxj.cross(nij)
                iMFac = 1 / (
                    pi.inverseMass
                    + pj.inverseMass
                    + rni.dot(rni) * pi.inverseMomentOfInertia
                    + rnj.dot(rnj) * pj.inverseMomentOfInertia
                )
                dpDelta = uij * iMFac
                dpNew = dp + dpDelta
                con[0] = dpNew

                pi.vel += dpDelta * pi.inverseMass
                pj.vel -= dpDelta * pj.inverseMass

                pi.omega += (dxi.cross(dpDelta)).y * pi.inverseMomentOfInertia
                pj.omega -= (dxj.cross(dpDelta)).y * pj.inverseMomentOfInertia

            for contact in collision_contacts:
                dpScalar, dpFric, pi, pj, dclose, xclose, nij, uij0, vnormal, dxi, dxj, iMFac = contact
                ui = pi.vel + Vec3(0.0, pi.omega, 0.0).cross(dxi)
                uj = pj.vel + Vec3(0.0, pj.omega, 0.0).cross(dxj)
                uij = uj - ui
                unormal = uij.dot(nij)
                vbias = self.collision_vbias(dclose, unormal)
                delta_dpScalar = (unormal - (vnormal + vbias)) * iMFac
                dpScalarNew = dpScalar + delta_dpScalar
                if dpScalarNew > 0.0:
                    dpScalarNew = 0.0
                delta_dpScalar = dpScalarNew - dpScalar
                contact[0] = dpScalarNew
                delta_dpFric = Vec3(0, 0, 0)
                if self.doFriction:
                    FricVecij = uij - nij * uij.dot(nij)
                    FricDir = FricVecij.normalized()
                    rni = dxi.cross(FricDir)
                    rnj = dxj.cross(FricDir)
                    iMFacFric = 1 / (
                        pi.inverseMass
                        + pj.inverseMass
                        + rni.dot(rni) * pi.inverseMomentOfInertia
                        + rnj.dot(rnj) * pj.inverseMomentOfInertia
                    )

                    delta_dpFric = FricVecij * iMFacFric
                    dpFricNew = dpFric + delta_dpFric
                    FricRatio = dpFricNew.length() / (abs(dpScalarNew) + 1e-20)
                    if FricRatio > self.muFrictionStatic:
                        dpFricNew = dpFricNew * self.muFrictionKinetic / FricRatio
                        delta_dpFric = dpFricNew - dpFric
                    contact[1] = dpFricNew

                pi.vel += (nij * delta_dpScalar + delta_dpFric) * pi.inverseMass
                pj.vel -= (nij * delta_dpScalar + delta_dpFric) * pj.inverseMass

                pi.omega += (dxi.cross(nij * delta_dpScalar + delta_dpFric)).y * pi.inverseMomentOfInertia
                pj.omega -= (dxj.cross(nij * delta_dpScalar + delta_dpFric)).y * pj.inverseMomentOfInertia

        if len(collision_contacts) + len(constraint_contacts) > 0:
            self.iColl += 1
            if self.fixPos:
                for p in self.RigidBody2Ds:
                    p.setPos(p.rold + (p.vel + p.vold) * 0.5 * dt)
                    p.theta = p.thetaold + (p.omega + p.omegaold) * 0.5 * dt
                    p.setHpr(90.0, p.theta * 180 / math.pi, 0.0)

        self.update_pendulum_period()
        self.capture_history_sample()
        return task.cont

    def report(self):
        summary = {
            "scenario": self.scenario,
            "time": self.t,
            "bias_mode": self.biasMode,
            "bodies": [self.body_snapshot(p) for p in self.RigidBody2Ds],
        }
        if self.scenario == 2:
            measured = self.average_pendulum_period()
            theory = 2.0 * math.pi * math.sqrt(2.0 / self.g)
            print("pendulum_crossings=%s" % self.pendulumCrossingTimes)
            print("pendulum_period_measured=%s" % ("%.6f" % measured if measured is not None else "None"))
            print("pendulum_period_theory=%.6f" % theory)
            summary["pendulum_crossings"] = list(self.pendulumCrossingTimes)
            summary["pendulum_period_measured"] = measured
            summary["pendulum_period_theory"] = theory
        elif self.scenario == 3:
            print("cradle_moving=%s" % self.moving_cradle_bodies())
            print("cradle_states=%s" % [(p.name, round(p.getPos().x, 4), round(p.vel.x, 4)) for p in self.cradleBodies])
            summary["cradle_moving"] = self.moving_cradle_bodies()
            summary["cradle_gap"] = self.cradleGap
            summary["cradle_rest_x"] = list(self.cradleRestX)
        elif self.scenario == 4:
            print(
                "stack_heights=%s"
                % [(p.name, round(p.getPos().z, 4), round(p.vel.z, 4)) for p in self.RigidBody2Ds if p.name.startswith("stack")]
            )
            summary["stack_heights"] = [
                {"name": p.name, "z": float(p.getPos().z), "vz": float(p.vel.z)}
                for p in self.RigidBody2Ds
                if p.name.startswith("stack")
            ]
        self.capture_history_sample(force=True)
        if self.summaryJsonPath:
            with open(self.summaryJsonPath, "w", encoding="utf-8") as fh:
                json.dump(summary, fh, indent=2)
        if self.historyJsonPath:
            with open(self.historyJsonPath, "w", encoding="utf-8") as fh:
                json.dump({"summary": summary, "samples": self.history}, fh, indent=2)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", type=int, default=DEFAULT_SCENARIO)
    parser.add_argument("--nsphere", type=int, default=DEFAULT_NSPHERE)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--offscreen", action="store_true")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--bias-mode", choices=["catto", "zero", "adaptive"], default=DEFAULT_BIAS_MODE)
    parser.add_argument("--collision-bias", type=float, default=0.3)
    parser.add_argument("--constraint-bias", type=float, default=0.05)
    parser.add_argument("--bias-speed-threshold", type=float, default=0.75)
    parser.add_argument("--pendulum-vx", type=float, default=-2.0)
    parser.add_argument("--cradle-gap", type=float, default=DEFAULT_CRADLE_GAP)
    parser.add_argument("--screenshot", type=str, default="")
    parser.add_argument("--summary-json", type=str, default="")
    parser.add_argument("--history-json", type=str, default="")
    parser.add_argument("--history-stride", type=int, default=1)
    return parser.parse_args()


def main():
    args = parse_args()
    scene = SimpleScene(args)
    if args.headless or args.offscreen:
        nsteps = max(1, int(args.duration / scene.dt))
        for _ in range(nsteps):
            scene.taskMgr.step()
        if args.screenshot and scene.win is not None:
            scene.graphicsEngine.renderFrame()
            scene.win.saveScreenshot(Filename.fromOsSpecific(args.screenshot))
        scene.report()
        scene.destroy()
        return
    scene.run()


if __name__ == "__main__":
    main()
