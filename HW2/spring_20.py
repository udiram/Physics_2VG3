import math
import os
from direct.showbase.ShowBase import ShowBase
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import DirectionalLight, TextNode, Vec3


class Particle:
    def __init__(self, node):
        self.node = node
        self.vel = Vec3(0.0, 0.0, 0.0)
        self.inverse_mass = 1.0
        self.force = Vec3(0.0, 0.0, 0.0)


class Spring20Demo(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        dlight = DirectionalLight("dlight")
        dlnp = self.render.attachNewNode(dlight)
        dlnp.setPos(50, 0, 100)
        dlnp.lookAt(0, 50, 0)
        self.render.setLight(dlnp)

        self.spring_length = 15.0
        self.spring_k = 10.0
        self.damping_d = 0.5
        self.target_sep = 20.0

        # v for circular-ish orbit at target sep: k(r-L) = mv^2/(r/2) -> v = sqrt(k(r-L)r/2)
        self.required_speed = math.sqrt(self.spring_k * (self.target_sep - self.spring_length) * self.target_sep / 2.0)

        hw2_dir = os.path.dirname(os.path.abspath(__file__))
        sphere = self.loader.loadModel(os.path.join(hw2_dir, "sphere.egg.pz"))
        self.cylinder_model = self.loader.loadModel(os.path.join(hw2_dir, "Cylinder.egg"))
        self.cylinder_model.setHpr(0, 90, 0)

        self.spring_container = self.render.attachNewNode("spring_container")
        self.spring_container.setPos(0.0, 100.0, 0.0)
        self.cylinder_model.reparentTo(self.spring_container)
        self.cylinder_model.setColor(0.9, 0.9, 0.9, 1.0)

        # two masses spinning around center
        self.particles = []
        start_positions = [Vec3(-10.0, 100.0, 0.0), Vec3(10.0, 100.0, 0.0)]
        start_velocities = [Vec3(0.0, 0.0, self.required_speed), Vec3(0.0, 0.0, -self.required_speed)]
        colors = [(1.0, 0.25, 0.25, 1.0), (0.25, 0.25, 1.0, 1.0)]
        for i in range(2):
            p_np = self.render.attachNewNode("particle")
            sphere.instanceTo(p_np)
            p_np.setColor(*colors[i])
            p_np.setPos(start_positions[i])
            p = Particle(p_np)
            p.vel = start_velocities[i]
            self.particles.append(p)

        OnscreenText(text=f"spring_20.py  |  L={self.spring_length:.1f}, k={self.spring_k:.1f}, target sep={self.target_sep:.1f}",
                     pos=(-1.3, 0.92), scale=0.05, fg=(1, 1, 1, 1), align=TextNode.ALeft)
        OnscreenText(text=f"Init speed for orbit: v = {self.required_speed:.4f}",
                     pos=(-1.3, 0.85), scale=0.05, fg=(1, 1, 1, 1), align=TextNode.ALeft)

        self.taskMgr.add(self.update_particles, "update_particles")
        print(f"velocity for sep=20: {self.required_speed:.6f}")

    def update_particles(self, task):
        dt = min(globalClock.getDt(), 1.0 / 120.0)

        p0 = self.particles[0]
        p1 = self.particles[1]

        r01 = p1.node.getPos() - p0.node.getPos()
        dist = r01.length()
        if dist < 1e-8:
            return task.cont
        n = r01 / dist

        F_spring = self.spring_k * (dist - self.spring_length)
        rel_vel = (p1.vel - p0.vel).dot(n)
        F_damp = self.damping_d * rel_vel
        total_force = n * (F_spring + F_damp)
        p0.force = total_force
        p1.force = -total_force

        for p in self.particles:
            accel = p.force * p.inverse_mass
            p.vel += accel * dt
            p.node.setPos(p.node.getPos() + p.vel * dt)

        # draw spring
        self.spring_container.setPos((p0.node.getPos() + p1.node.getPos()) * 0.5)
        self.spring_container.lookAt(p0.node.getPos())
        self.cylinder_model.setScale(0.1, 0.1, dist * 0.275)

        return task.cont


if __name__ == "__main__":
    Spring20Demo().run()
