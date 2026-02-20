import os
from direct.showbase.ShowBase import ShowBase
from panda3d.core import *

_ASSETS = os.path.dirname(os.path.abspath(__file__))

position_vectors = [
    Vec3(-11, 200, 0),
    Vec3(0, 200, 0),
    Vec3(120, 200, 0)]

velocity_vectors = [
    Vec3(10, 0, 0),
    Vec3(10, 0, 0),
    Vec3(0, 0, 0)]

radii = [1, 10, 100]

inverse_masses = [1e9, 0.01, 0]  # mass = 1, 100, infinite

# colliding particle -- has bounding sphere
class Particle(NodePath):
    def __init__(self, *args, **kwargs):
        NodePath.__init__(self, *args, **kwargs)
        self.vel = Vec3(0., 0., 0.)
        self.inverseMass = 1.0
        self.radius = 1.0


class SimpleScene(ShowBase):

    def calculateConservation(self, label="", about_point=None):
        """
        Prints total linear momentum, angular momentum, and kinetic energy
        for all finite-mass particles.

        label: optional string to tag the printout ("before", "after", etc.)
        about_point: Vec3 point to compute angular momentum about (default: origin)
        """
        if about_point is None:
            about_point = Vec3(0.0, 0.0, 0.0)

        P_tot = Vec3(0.0, 0.0, 0.0)
        L_tot = Vec3(0.0, 0.0, 0.0)
        K_tot = 0.0

        for p in self.particles:
            inv_m = getattr(p, "inverseMass", 0.0)

            # skip infinite-mass objects (e.g., wall)
            if inv_m <= 0.0:
                continue

            m = 1.0 / inv_m

            v = p.vel
            r = p.getPos() - about_point

            p_lin = v * m  # linear momentum of this particle
            P_tot += p_lin

            L_tot += r.cross(p_lin)  # angular momentum about about_point

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

        if (True):
            dlight = DirectionalLight('dlight')
            dlnp = render.attachNewNode(dlight)
            dlnp.setPos(20, 0, 40)
            dlnp.lookAt(0, 0, 0)
            render.setLight(dlnp)

        self.sphere = loader.loadModel("./panda3d/sphere.egg.pz")

        # Initialize particles
        self.particles = []
        for i in range(3):
            p = Particle("p")
            self.sphere.instanceTo(p)

            # Adjust physical and rendering properties of particle here
            p.setPos(position_vectors[i])
            p.vel = velocity_vectors[i]
            p.radius = radii[i]
            p.inverseMass = inverse_masses[i]
            p.setScale(p.radius)
            p.reparentTo(render)
            self.particles.append(p)

        self.gameTask = taskMgr.add(self.updateParticles, "updateParticles")
        self.calculateConservation("before collision")

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

                        # print after the second collision (wall bounce then ball-ball)
                        if (self._collision_count == 2) and (not self._printed_after):
                            print("\nVelocities [after 2nd collision]:")
                            for k, p in enumerate(self.particles):
                                print(f"  p{k}: v = ({p.vel.x}, {p.vel.y}, {p.vel.z})")
                            self._printed_after = True
                            self.calculateConservation("after 2nd collision")

        return task.cont


scene = SimpleScene()
scene.run()