import math

from panda3d.core import NodePath, Vec3


class Particle(NodePath):
    def __init__(
        self,
        loader,
        parent,
        pos,
        vel,
        inverseMass=1.0,
        radius=1.0,
        color=(1.0, 1.0, 1.0, 1.0),
        name="particle",
    ):
        NodePath.__init__(self, name)
        self.reparentTo(parent)

        model = loader.loadModel("models/misc/sphere")
        model.reparentTo(self)

        self.setScale(radius)
        self.setColor(*color)
        self.setPos(pos)

        self.vel = Vec3(vel)
        self.inverseMass = inverseMass
        self.radius = radius

    @property
    def mass(self):
        if self.inverseMass == 0.0:
            return float("inf")
        return 1.0 / self.inverseMass


class CollisionWorld:
    def __init__(self, particles, restitution=1.0, substeps=4, solver_iterations=4):
        self.particles = list(particles)
        self.restitution = restitution
        self.substeps = substeps
        if self.substeps < 1:
            self.substeps = 1
        self.solver_iterations = solver_iterations
        if self.solver_iterations < 1:
            self.solver_iterations = 1
        self.time = 0.0
        self.collision_count = 0

    def step(self, dt):
        hits = 0
        small_dt = dt / float(self.substeps)

        for _ in range(self.substeps):
            hits += self._do_substep(small_dt)

        self.time += dt
        return hits

    def _do_substep(self, dt):
        time_left = dt
        hits = 0
        tiny = 1e-8
        loops = 0
        max_loops = max(16, len(self.particles) * 4)

        while time_left > tiny and loops < max_loops:
            hit_t, pairs = self._find_first_hit(time_left)
            self._move_all(hit_t)
            time_left -= hit_t

            if not pairs:
                break

            counted = set()
            for _ in range(self.solver_iterations):
                changed = False
                for i, j in pairs:
                    if self._resolve_pair(self.particles[i], self.particles[j]):
                        changed = True
                        if (i, j) not in counted:
                            counted.add((i, j))
                            hits += 1
                            self.collision_count += 1
                if not changed:
                    break

            loops += 1

            if hit_t <= tiny and not counted and time_left > tiny:
                # small push forward so the same contact does not repeat forever
                nudge = min(time_left, 1e-6)
                self._move_all(nudge)
                time_left -= nudge

        if time_left > tiny:
            self._move_all(time_left)

        return hits

    def _move_all(self, dt):
        if dt <= 0.0:
            return

        for p in self.particles:
            p.setPos(p.getPos() + p.vel * dt)

    def _find_first_hit(self, max_t):
        best_t = max_t
        pairs = []
        tol = 1e-7

        n = len(self.particles)
        for i in range(n):
            for j in range(i + 1, n):
                t = self._time_of_impact(self.particles[i], self.particles[j], max_t)
                if t is None:
                    continue

                if t < best_t - tol:
                    best_t = t
                    pairs = [(i, j)]
                elif abs(t - best_t) <= tol:
                    pairs.append((i, j))

        if pairs:
            return best_t, pairs
        return max_t, []

    def _time_of_impact(self, a, b, max_t):
        dp = b.getPos() - a.getPos()
        dv = b.vel - a.vel
        r = a.radius + b.radius

        c = dp.dot(dp) - r * r
        if c <= 1e-8:
            return 0.0

        aa = dv.dot(dv)
        if aa <= 1e-12:
            return None

        bb = 2.0 * dp.dot(dv)
        if bb >= 0.0:
            return None

        disc = bb * bb - 4.0 * aa * c
        if disc < 0.0:
            return None

        t = (-bb - math.sqrt(disc)) / (2.0 * aa)
        if t < -1e-8 or t > max_t + 1e-8:
            return None

        if t < 0.0:
            return 0.0
        return t

    def calculateConservation(self, about=Vec3(0.0, 0.0, 0.0), include_infinite=False):
        total_p = Vec3(0.0, 0.0, 0.0)
        total_l = Vec3(0.0, 0.0, 0.0)
        total_k = 0.0

        for p in self.particles:
            if p.inverseMass == 0.0 and not include_infinite:
                continue

            m = p.mass
            momentum = p.vel * m
            total_p += momentum
            total_l += (p.getPos() - about).cross(momentum)
            total_k += 0.5 * m * p.vel.dot(p.vel)

        out = {
            "time": self.time,
            "P": total_p,
            "L": total_l,
            "K": total_k,
            "collisions": self.collision_count,
        }

        print(f"t={out['time']}")
        print(f"  linear momentum P = {out['P']}")
        print(f"  angular momentum L = {out['L']}")
        print(f"  kinetic energy K = {out['K']}")
        print(f"  total collisions so far = {out['collisions']}")

        return out

    def bounce_in_box(self, particle, min_corner, max_corner):
        pos = particle.getPos()

        if pos.x - particle.radius < min_corner.x:
            pos.x = min_corner.x + particle.radius
            particle.vel.x = -particle.vel.x
        elif pos.x + particle.radius > max_corner.x:
            pos.x = max_corner.x - particle.radius
            particle.vel.x = -particle.vel.x

        if pos.y - particle.radius < min_corner.y:
            pos.y = min_corner.y + particle.radius
            particle.vel.y = -particle.vel.y
        elif pos.y + particle.radius > max_corner.y:
            pos.y = max_corner.y - particle.radius
            particle.vel.y = -particle.vel.y

        if pos.z - particle.radius < min_corner.z:
            pos.z = min_corner.z + particle.radius
            particle.vel.z = -particle.vel.z
        elif pos.z + particle.radius > max_corner.z:
            pos.z = max_corner.z - particle.radius
            particle.vel.z = -particle.vel.z

        particle.setPos(pos)

    def _resolve_pair(self, a, b):
        delta = b.getPos() - a.getPos()
        min_dist = a.radius + b.radius
        dist_sq = delta.dot(delta)

        if dist_sq > (min_dist + 1e-6) * (min_dist + 1e-6):
            return False

        dist = math.sqrt(dist_sq)
        if dist > 1e-10:
            normal = delta / dist
            overlap = min_dist - dist
        else:
            rel = b.vel - a.vel
            if rel.lengthSquared() > 1e-12:
                normal = rel.normalized()
            else:
                normal = Vec3(1.0, 0.0, 0.0)
            overlap = min_dist

        inv_mass_sum = a.inverseMass + b.inverseMass
        if inv_mass_sum <= 0.0:
            return False

        if overlap > 0.0:
            move = normal * (overlap / inv_mass_sum)
            a.setPos(a.getPos() - move * a.inverseMass)
            b.setPos(b.getPos() + move * b.inverseMass)

        rel_vel = b.vel - a.vel
        speed_along_normal = rel_vel.dot(normal)
        if speed_along_normal >= -1e-9:
            return False

        j = -(1.0 + self.restitution) * speed_along_normal / inv_mass_sum
        impulse = normal * j

        a.vel -= impulse * a.inverseMass
        b.vel += impulse * b.inverseMass
        return True
