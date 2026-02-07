from direct.showbase.ShowBase import ShowBase
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import DirectionalLight, TextNode, Vec3
import math
import os


class Particle:
    def __init__(self, node):
        self.node = node
        self.vel = Vec3(0.0, 0.0, 0.0)
        self.inverse_mass = 1.0
        self.force = Vec3(0.0, 0.0, 0.0)


class SpringCriticalDemo(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        dlight = DirectionalLight("dlight")
        dlnp = self.render.attachNewNode(dlight)
        dlnp.setPos(50, 0, 100)
        dlnp.lookAt(0, 50, 0)
        self.render.setLight(dlnp)

        self.spring_length = 15.0
        self.spring_k = 1.0

        # For two equal masses m=1 connected by one spring:
        # e'' + 2 D e' + 2 k e = 0  ->  D_critical = sqrt(2 k)
        self.damping_d = math.sqrt(2.0 * self.spring_k)

        hw2_dir = os.path.dirname(os.path.abspath(__file__))
        sphere = self.loader.loadModel(os.path.join(hw2_dir, "sphere.egg.pz"))

        self.particles = []
        start_positions = [Vec3(-20.0, 100.0, 0.0), Vec3(20.0, 100.0, 0.0)]
        colors = [(1.0, 0.2, 0.2, 1.0), (0.2, 0.2, 1.0, 1.0)]
        for i in range(2):
            p_np = self.render.attachNewNode("particle")
            sphere.instanceTo(p_np)
            p_np.setColor(*colors[i])
            p_np.setPos(start_positions[i])
            p = Particle(p_np)
            self.particles.append(p)

        OnscreenText(
            text=f"spring_critical.py  |  k={self.spring_k:.1f}, L={self.spring_length:.1f}, D={self.damping_d:.4f}",
            pos=(-1.3, 0.92),
            scale=0.05,
            fg=(1, 1, 1, 1),
            align=TextNode.ALeft,
        )
        OnscreenText(
            text="Initial condition: x=-20 and x=20, zero initial velocity",
            pos=(-1.3, 0.85),
            scale=0.045,
            fg=(1, 1, 1, 1),
            align=TextNode.ALeft,
        )

        self.taskMgr.add(self.update_particles, "update_particles")
        print(f"Critical damping coefficient D = sqrt(2k) = {self.damping_d:.6f}")

    def update_particles(self, task):
        dt = min(globalClock.getDt(), 1.0 / 120.0)

        p0 = self.particles[0]
        p1 = self.particles[1]

        r01 = p1.node.getPos() - p0.node.getPos()
        dist = r01.length()
        if dist < 1e-8:
            return task.cont
        n01 = r01 / dist

        spring_force_mag = self.spring_k * (dist - self.spring_length)
        relative_speed = (p1.vel - p0.vel).dot(n01)
        damping_force_mag = self.damping_d * relative_speed

        total_force = n01 * (spring_force_mag + damping_force_mag)
        p0.force = total_force
        p1.force = -total_force

        for p in self.particles:
            accel = p.force * p.inverse_mass
            p.vel += accel * dt
            p.node.setPos(p.node.getPos() + p.vel * dt)

        return task.cont


if __name__ == "__main__":
    app = SpringCriticalDemo()
    app.run()
