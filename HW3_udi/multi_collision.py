import math
from panda3d.core import NodePath, Vec3


class Particle(NodePath):
    def __init__(self, loader, parent, pos, vel, inverseMass=1.0, radius=1.0,
                 color=(1.0, 1.0, 1.0, 1.0), name="particle"):
        super().__init__(name)
        self.reparentTo(parent)

        visual = loader.loadModel("models/misc/sphere")
        visual.reparentTo(self)
        self.setScale(radius)
        self.setColor(*color)
        self.setPos(pos)

        self.vel = Vec3(vel)
        self.inverseMass = inverseMass
        self.radius = radius

    @property
    def mass(self) -> float:
        return float("inf") if self.inverseMass == 0.0 else 1.0 / self.inverseMass


class CollisionWorld:
    def __init__(self, particles, restitution=1.0, substeps=4, solver_iterations=4):
        self.particles = list(particles)
        self.restitution = restitution
        self.substeps = max(1, substeps)
        self.solver_iterations = max(1, solver_iterations)
        self.time = 0.0
        self.collision_count = 0

    def step(self, dt):
        hits = 0
        sub_dt = dt / float(self.substeps)

        for _ in range(self.substeps):
            hits += self._integrate_substep(sub_dt)

        self.time += dt
        return hits

    def _integrate_substep(self, dt):
        # handle sphere-sphere collisions for this timestep
        remaining = dt
        hits = 0
        eps = 1e-8
        max_events = max(16, len(self.particles) * 4)
        events = 0

        while remaining > eps and events < max_events:
            hit_time, hit_pairs = self._find_earliest_impact(remaining)
            self._advance_all(hit_time)
            remaining -= hit_time

            if not hit_pairs:
                break

            counted = set()
            for _ in range(self.solver_iterations):
                changed = False
                for i, j in hit_pairs:
                    if self._resolve_pair(self.particles[i], self.particles[j]):
                        changed = True
                        key = (i, j)
                        if key not in counted:
                            counted.add(key)
                            hits += 1
                            self.collision_count += 1
                if not changed:
                    break

            events += 1

            # avoid getting stuck on repeated t=0 hits
            if hit_time <= eps and not counted and remaining > eps:
                nudge = min(remaining, 1e-6)
                self._advance_all(nudge)
                remaining -= nudge

        if remaining > eps:
            self._advance_all(remaining)

        return hits

    def _advance_all(self, dt):
        if dt <= 0.0:
            return
        for particle in self.particles:
            particle.setPos(particle.getPos() + particle.vel * dt)

    def _find_earliest_impact(self, max_t):
        toi_eps = 1e-7
        earliest = max_t
        pairs: list[tuple[int, int]] = []

        for i in range(len(self.particles)):
            for j in range(i + 1, len(self.particles)):
                toi = self._time_of_impact(self.particles[i], self.particles[j], max_t)
                if toi is None:
                    continue
                if toi < earliest - toi_eps:
                    earliest = toi
                    pairs = [(i, j)]
                elif abs(toi - earliest) <= toi_eps:
                    pairs.append((i, j))

        if not pairs:
            return max_t, []
        return earliest, pairs

    def _time_of_impact(self, a, b, max_t):
        rel_pos = b.getPos() - a.getPos()
        rel_vel = b.vel - a.vel
        target = a.radius + b.radius
        target_sq = target * target

        c = rel_pos.dot(rel_pos) - target_sq
        if c <= 1e-8:
            return 0.0

        aa = rel_vel.dot(rel_vel)
        if aa <= 1e-12:
            return None

        bb = 2.0 * rel_pos.dot(rel_vel)
        if bb >= 0.0:
            return None

        disc = bb * bb - 4.0 * aa * c
        if disc < 0.0:
            return None

        t = (-bb - math.sqrt(disc)) / (2.0 * aa)
        if t < -1e-8 or t > max_t + 1e-8:
            return None
        return max(0.0, min(max_t, t))

    def calculateConservation(self, about=Vec3(0.0, 0.0, 0.0), include_infinite=False):
        p_total = Vec3(0.0, 0.0, 0.0)
        l_total = Vec3(0.0, 0.0, 0.0)
        k_total = 0.0

        for particle in self.particles:
            if particle.inverseMass == 0.0 and not include_infinite:
                continue
            mass = particle.mass
            momentum = particle.vel * mass
            p_total += momentum
            r = particle.getPos() - about
            l_total += r.cross(momentum)
            k_total += 0.5 * mass * particle.vel.dot(particle.vel)

        out = {
            "time": self.time,
            "P": p_total,
            "L": l_total,
            "K": k_total,
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
            particle.vel.x *= -1.0
        elif pos.x + particle.radius > max_corner.x:
            pos.x = max_corner.x - particle.radius
            particle.vel.x *= -1.0

        if pos.y - particle.radius < min_corner.y:
            pos.y = min_corner.y + particle.radius
            particle.vel.y *= -1.0
        elif pos.y + particle.radius > max_corner.y:
            pos.y = max_corner.y - particle.radius
            particle.vel.y *= -1.0

        if pos.z - particle.radius < min_corner.z:
            pos.z = min_corner.z + particle.radius
            particle.vel.z *= -1.0
        elif pos.z + particle.radius > max_corner.z:
            pos.z = max_corner.z - particle.radius
            particle.vel.z *= -1.0

        particle.setPos(pos)

    def _resolve_pair(self, a, b):
        delta = b.getPos() - a.getPos()
        min_dist = a.radius + b.radius
        dist_sq = delta.dot(delta)
        tol = 1e-6

        if dist_sq > (min_dist + tol) * (min_dist + tol):
            return False

        dist = dist_sq ** 0.5
        if dist > 1e-10:
            normal = delta / dist
            penetration = min_dist - dist
        else:
            rel = b.vel - a.vel
            if rel.lengthSquared() > 1e-12:
                normal = rel.normalized()
            else:
                normal = Vec3(1.0, 0.0, 0.0)
            penetration = min_dist

        inv_mass_sum = a.inverseMass + b.inverseMass
        if inv_mass_sum <= 0.0:
            return False

        # push apart so they dont overlap
        if penetration > 0.0:
            correction = normal * (penetration / inv_mass_sum)
            a.setPos(a.getPos() - correction * a.inverseMass)
            b.setPos(b.getPos() + correction * b.inverseMass)

        rel_vel = b.vel - a.vel
        vel_along_normal = rel_vel.dot(normal)

        if vel_along_normal >= -1e-9:
            return False

        impulse_mag = -(1.0 + self.restitution) * vel_along_normal / inv_mass_sum
        impulse = normal * impulse_mag

        a.vel -= impulse * a.inverseMass
        b.vel += impulse * b.inverseMass
        return True


class _CollisionDemoApp:
    def __init__(self):
        from direct.showbase.ShowBase import ShowBase
        from direct.showbase.ShowBaseGlobal import globalClock
        from direct.task import Task
        from panda3d.core import AmbientLight, DirectionalLight

        class Demo(ShowBase):
            def __init__(self):
                super().__init__()
                self.disableMouse()

                if self.camera is not None:
                    self.camera.setPos(0.0, -36.0, 8.0)
                    self.camera.lookAt(0.0, 25.0, 0.0)

                ambient = AmbientLight("ambient")
                ambient.setColor((0.75, 0.75, 0.75, 1.0))
                self.render.setLight(self.render.attachNewNode(ambient))

                sun = DirectionalLight("sun")
                sun.setColor((0.6, 0.6, 0.6, 1.0))
                sun_np = self.render.attachNewNode(sun)
                sun_np.setHpr(-20.0, -35.0, 0.0)
                self.render.setLight(sun_np)

                import random

                rng = random.Random(15)
                palette = [
                    (0.20, 0.70, 1.00, 1.0),
                    (1.00, 0.45, 0.25, 1.0),
                    (0.35, 1.00, 0.40, 1.0),
                    (1.00, 0.90, 0.25, 1.0),
                    (0.90, 0.30, 1.00, 1.0),
                    (0.45, 1.00, 0.95, 1.0),
                ]

                self.particles = []
                idx = 0
                for z in (-4.0, 0.0, 4.0):
                    for x in (-12.0, -6.0, 0.0, 6.0, 12.0):
                        while True:
                            vx = rng.uniform(-9.0, 9.0)
                            vz = rng.uniform(-5.0, 5.0)
                            if abs(vx) + abs(vz) > 4.0:
                                break

                        self.particles.append(
                            Particle(
                                loader=self.loader,
                                parent=self.render,
                                pos=Vec3(x, 25.0, z),
                                vel=Vec3(vx, 0.0, vz),
                                inverseMass=1.0,
                                radius=1.0,
                                color=palette[idx % len(palette)],
                                name=f"demo_{idx + 1}",
                            )
                        )
                        idx += 1

                self.world = CollisionWorld(self.particles, restitution=1.0, substeps=6, solver_iterations=6)
                print("=== Conserved quantities at start (t=0) ===")
                self.world.calculateConservation()
                self.reported_at_30 = False
                self.min_corner = Vec3(-16.0, 22.0, -8.0)
                self.max_corner = Vec3(16.0, 28.0, 8.0)
                self.fixed_dt = 1.0 / 240.0
                self.accumulator = 0.0
                self.max_steps_per_frame = 32
                self.next_report = 1.0

                self._globalClock = globalClock
                self.taskMgr.add(self.update_sim, "demo_update")

            def update_sim(self, task):
                frame_dt = min(self._globalClock.getDt(), 0.1)
                if frame_dt <= 0.0:
                    return Task.cont

                self.accumulator += frame_dt
                steps = 0
                while self.accumulator >= self.fixed_dt and steps < self.max_steps_per_frame:
                    self.world.step(self.fixed_dt)
                    for particle in self.particles:
                        self.world.bounce_in_box(particle, self.min_corner, self.max_corner)
                    self.accumulator -= self.fixed_dt
                    steps += 1

                if self.world.time >= self.next_report:
                    speed_sum = sum(p.vel.length() for p in self.particles)
                    print(f"t={self.world.time:.2f}s collisions={self.world.collision_count} total_speed={speed_sum:.3f}")
                    self.next_report += 1.0

                if self.world.time >= 30.0 and not self.reported_at_30:
                    print("=== Conserved quantities after 30 seconds ===")
                    self.world.calculateConservation()
                    self.reported_at_30 = True

                return Task.cont

        self.app = Demo()

    def run(self):
        print("Running Panda3D collision demo (close window or Ctrl+C to quit).")
        self.app.run()


if __name__ == "__main__":
    _CollisionDemoApp().run()
