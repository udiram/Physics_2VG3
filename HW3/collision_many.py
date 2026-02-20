import os
import random
from direct.showbase.ShowBase import ShowBase
from panda3d.core import *

_ASSETS = os.path.dirname(os.path.abspath(__file__))


class Particle(NodePath):
    def __init__(self, *args, **kwargs):
        NodePath.__init__(self, *args, **kwargs)
        self.vel = Vec3(0.0, 0.0, 0.0)
        self.inverseMass = 1.0
        self.radius = 1.0


class SimpleScene(ShowBase):
    def calculateConservation(self, label="", about_point=None):
        if about_point is None:
            about_point = Vec3(0.0, 0.0, 0.0)

        P_tot = Vec3(0.0, 0.0, 0.0)
        L_tot = Vec3(0.0, 0.0, 0.0)
        K_tot = 0.0

        for p in self.particles:
            inv_m = getattr(p, "inverseMass", 0.0)
            if inv_m <= 0.0:
                continue

            m = 1.0 / inv_m
            v = p.vel
            r = p.getPos() - about_point

            p_lin = v * m
            P_tot += p_lin
            L_tot += r.cross(p_lin)
            K_tot += 0.5 * m * v.lengthSquared()

        tag = f" [{label}]" if label else ""
        print(f"\nConserved quantities{tag} (about {about_point}):")
        print(f"  P = ({P_tot.x}, {P_tot.y}, {P_tot.z})")
        print(f"  L = ({L_tot.x}, {L_tot.y}, {L_tot.z})")
        print(f"  K = {K_tot}")

    def __init__(self):
        super().__init__(self)

        self.g = 0.0
        self.eCoeffRestitution = 1.0
        self._printed_after = False
        self._collision_count = 0

        self.center = Vec3(0.0, 100.0, 0.0)
        self.half_size = 20.0
        self.wall_min = self.center - Vec3(self.half_size, self.half_size, self.half_size)
        self.wall_max = self.center + Vec3(self.half_size, self.half_size, self.half_size)

        dlight = DirectionalLight('dlight')
        dlnp = render.attachNewNode(dlight)
        dlnp.setPos(20, 0, 40)
        dlnp.lookAt(0, 0, 0)
        render.setLight(dlnp)

        self.sphere = loader.loadModel("./panda3d/sphere.egg.pz")

        self.particles = []
        num_particles = 10

        for _ in range(num_particles):
            p = Particle("p")
            self.sphere.instanceTo(p)

            p.radius = 1.0
            p.inverseMass = 1.0

            x = random.uniform(self.center.x - 18.0, self.center.x + 18.0)
            y = self.center.y
            z = random.uniform(self.center.z - 18.0, self.center.z + 18.0)
            p.setPos(Vec3(x, y, z))

            p.vel = Vec3(
                random.uniform(-10.0, 10.0),
                0.0,
                random.uniform(-10.0, 10.0),
            )

            p.setScale(p.radius)
            p.reparentTo(render)
            self.particles.append(p)

        self.gameTask = taskMgr.add(self.updateParticles, "updateParticles")
        self.calculateConservation("start")

    def _bounce_walls(self, p):
        min_x = self.wall_min.x
        max_x = self.wall_max.x
        min_y = self.wall_min.y
        max_y = self.wall_max.y
        min_z = self.wall_min.z
        max_z = self.wall_max.z

        pos = p.getPos()
        r = p.radius
        e = self.eCoeffRestitution

        if pos.x - r < min_x:
            pos.x = min_x + r
            p.vel.x = -p.vel.x * e
        elif pos.x + r > max_x:
            pos.x = max_x - r
            p.vel.x = -p.vel.x * e

        # Constrain motion to the XZ plane (lock Y) --> make collisions guaranteed to happen
        pos.y = self.center.y
        p.vel.y = 0.0

        if pos.z - r < min_z:
            pos.z = min_z + r
            p.vel.z = -p.vel.z * e
        elif pos.z + r > max_z:
            pos.z = max_z - r
            p.vel.z = -p.vel.z * e

        p.setPos(pos)

    def updateParticles(self, task):
        # Get the time elapsed since the next frame.

        # dt = globalClock.getDt()
        # use precise timesteps
        dt = 1 / 60.

        for p in self.particles:
            a = Vec3(0., 0., -self.g)  # simple a here
            vold = p.vel
            p.vel = p.vel + a * dt
            rold = p.getPos()
            r = rold + p.vel * dt
            p.setPos(r)
            self._bounce_walls(p)

        # check other particles in pairs (exclude self)
        for i, pi in enumerate(self.particles):
            for pj in self.particles[i + 1:]:
                # collision detection
                dpos = pj.getPos() - pi.getPos()
                d = dpos.length()
                r_sum = pi.radius + pj.radius

                # check 1: overlap
                if (d < r_sum) and (d > 0.0001):
                    # print("Overlap detected")

                    n_hat = dpos / d
                    rel_speed_n = (pi.vel - pj.vel).dot(n_hat)

                    # check 2: must be approaching
                    if (rel_speed_n > 0):
                        # print("Collision detected!")

                        # resolve collision
                        penetration = r_sum - d

                        # positional correction (split evenly)
                        shift = n_hat * (0.5 * penetration)
                        pi.setPos(pi.getPos() - shift)
                        pj.setPos(pj.getPos() + shift)

                        # impulse update
                        e = self.eCoeffRestitution
                        impulse = -(1.0 + e) * rel_speed_n / (pi.inverseMass + pj.inverseMass)

                        pi.vel += n_hat * impulse * pi.inverseMass
                        pj.vel -= n_hat * impulse * pj.inverseMass

                        # count resolved collisions
                        self._collision_count += 1

        if (not self._printed_after) and (task.time >= 30.0):
            self._printed_after = True
            self.calculateConservation("after 30s")

        return task.cont


scene = SimpleScene()
scene.run()
