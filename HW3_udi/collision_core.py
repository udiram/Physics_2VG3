import math
from dataclasses import dataclass


@dataclass
class Vec3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __add__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> "Vec3":
        return Vec3(self.x * scalar, self.y * scalar, self.z * scalar)

    __rmul__ = __mul__

    def __truediv__(self, scalar: float) -> "Vec3":
        return Vec3(self.x / scalar, self.y / scalar, self.z / scalar)

    def dot(self, other: "Vec3") -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other: "Vec3") -> "Vec3":
        return Vec3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )

    def norm(self) -> float:
        return math.sqrt(self.dot(self))

    def normalized(self) -> "Vec3":
        n = self.norm()
        if n == 0.0:
            return Vec3(0.0, 0.0, 0.0)
        return self / n

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def __repr__(self) -> str:
        return f"Vec3({self.x:.12g}, {self.y:.12g}, {self.z:.12g})"


class Particle:
    def __init__(self, pos, vel, inverseMass=1.0, radius=1.0, name=""):
        self.pos = pos
        self.vel = vel
        self.inverseMass = inverseMass
        self.radius = radius
        self.name = name

    @property
    def mass(self) -> float:
        if self.inverseMass == 0.0:
            return float("inf")
        return 1.0 / self.inverseMass


class CollisionWorld:
    def __init__(self, particles, restitution=1.0):
        self.particles = particles
        self.restitution = restitution
        self.time = 0.0
        self.collision_count = 0

    def step(self, dt: float) -> int:
        for p in self.particles:
            p.pos = p.pos + p.vel * dt

        events = 0
        n = len(self.particles)
        for i in range(n):
            for j in range(i + 1, n):
                if self._resolve_pair(self.particles[i], self.particles[j]):
                    events += 1
                    self.collision_count += 1

        self.time += dt
        return events

    def run(self, duration: float, dt: float) -> int:
        steps = int(duration / dt)
        events = 0
        for _ in range(steps):
            events += self.step(dt)
        return events

    def calculateConservation(self, about=Vec3(0.0, 0.0, 0.0), include_infinite=False):
        p_total = Vec3(0.0, 0.0, 0.0)
        l_total = Vec3(0.0, 0.0, 0.0)
        k_total = 0.0

        for p in self.particles:
            if p.inverseMass == 0.0 and not include_infinite:
                continue
            m = p.mass
            lin_mom = p.vel * m
            p_total = p_total + lin_mom
            r = p.pos - about
            l_total = l_total + r.cross(lin_mom)
            k_total += 0.5 * m * p.vel.dot(p.vel)

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

    def _resolve_pair(self, a, b):
        delta = b.pos - a.pos
        dist = delta.norm()
        min_dist = a.radius + b.radius

        if dist == 0.0:
            normal = Vec3(1.0, 0.0, 0.0)
            penetration = min_dist
        else:
            normal = delta / dist
            penetration = min_dist - dist

        if penetration < 0.0:
            return False

        relative_vel = b.vel - a.vel
        vel_along_normal = relative_vel.dot(normal)

        inv_mass_sum = a.inverseMass + b.inverseMass
        if inv_mass_sum == 0.0:
            return False

        if vel_along_normal > 0.0:
            self._positional_correction(a, b, normal, penetration, inv_mass_sum)
            return False

        j = -(1.0 + self.restitution) * vel_along_normal / inv_mass_sum
        impulse = normal * j

        a.vel = a.vel - impulse * a.inverseMass
        b.vel = b.vel + impulse * b.inverseMass

        self._positional_correction(a, b, normal, penetration, inv_mass_sum)
        return True

    def _positional_correction(self, a, b, normal, penetration, inv_mass_sum):
        if penetration <= 0.0:
            return
        correction = normal * (penetration / inv_mass_sum)
        a.pos = a.pos - correction * a.inverseMass
        b.pos = b.pos + correction * b.inverseMass


def format_vec(v):
    return f"({v.x}, {v.y}, {v.z})"
