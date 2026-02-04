from direct.showbase.ShowBase import ShowBase
from panda3d.core import *
import random
import math

# Faint firework colors (R, G, B) – soft, not saturated
FIREWORK_COLORS = [
    (0.55, 0.25, 0.25),   # faint red
    (0.6, 0.4, 0.2),     # faint gold
    (0.25, 0.4, 0.6),    # faint blue
    (0.25, 0.55, 0.35),  # faint green
    (0.7, 0.7, 0.65),    # faint white
    (0.55, 0.35, 0.5),   # faint pink
    (0.5, 0.5, 0.6),     # faint violet
]

# Cooled/aged spark color (particles shift toward this as they age)
COOLED_COLOR = (0.35, 0.2, 0.1)


class Particle(NodePath):
    def __init__(self, *args, **kwargs):
        NodePath.__init__(self, *args, **kwargs)
        self.vel = Vec3(0.0, 0.0, 0.0)
        self.inverseMass = 1.0


class ParticleLifetime(Particle):
    def __init__(self, *args, **kwargs):
        Particle.__init__(self, *args, **kwargs)
        self.tStart = globalClock.getRealTime()
        self.tStop = 1e37
        self.initialScale = 0.1
        self.color = (1, 1, 1)


class ParticleEmitter(ParticleLifetime):
    """Short-lived burst emitter. Stops emitting after emitDuration, then is removed (ParticleLifetime)."""
    def __init__(self, *args, **kwargs):
        ParticleLifetime.__init__(self, *args, **kwargs)
        self.emitAngle = 180
        self.emitRate = 0.0       # set when burst is created
        self.emitVel = 0.0
        self.emitLifetime = 2.0   # particle lifetime
        self.emitDir = Vec3(0, 0, -1)
        self.emitAccum = 0.0
        self.emitDuration = 0.2  # how long this emitter fires (short burst)
        self.tStop = self.tStart + self.emitDuration  # destroy emitter when done


def randomInCone(coneHalfAngleDeg, axis):
    """Return a unit vector in a cone around axis. coneHalfAngleDeg in degrees."""
    if coneHalfAngleDeg >= 180:
        # full sphere: uniform random direction
        while True:
            v = Vec3(
                random.uniform(-1, 1),
                random.uniform(-1, 1),
                random.uniform(-1, 1),
            )
            if v.length() > 0.001:
                return v.normalized()
    axis = axis.normalized()
    halfRad = math.radians(coneHalfAngleDeg)
    # random angle from axis in [0, halfRad], uniform in solid angle
    cosMax = math.cos(halfRad)
    cosTheta = random.uniform(cosMax, 1.0)
    theta = math.acos(cosTheta)
    phi = random.uniform(0, 2 * math.pi)
    # perpendicular unit vectors to axis
    if abs(axis.z) < 0.9:
        perp = Vec3(0, 0, 1).cross(axis).normalized()
    else:
        perp = Vec3(1, 0, 0).cross(axis).normalized()
    perp2 = axis.cross(perp).normalized()
    return (axis * math.cos(theta) +
            perp * (math.sin(theta) * math.cos(phi)) +
            perp2 * (math.sin(theta) * math.sin(phi)))


class SimpleScene(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        self.g = 9.8
        self.sphere = loader.loadModel("sphere.egg.pz")
        self.particles = []
        self.particleEmitters = []

        # Dark sky background
        self.setBackgroundColor(0.05, 0.05, 0.15, 1.0)

        # Camera: look at the firework area from a nice angle
        self.cam.setPos(0, 20, 35)
        self.cam.lookAt(0, 50, 0)

        self.nextBurstTime = globalClock.getRealTime()
        # Start with a couple of bursts right away
        self._addBurst()
        self.nextBurstTime = globalClock.getRealTime() + random.uniform(0.3, 0.8)

        self.gameTask = taskMgr.add(self.updateParticles, "updateParticles")
        self.emitTask = taskMgr.add(self.updateParticleEmitters, "updateEmitters")
        self.spawnTask = taskMgr.add(self.spawnBurstEmitters, "spawnEmitters")

    def _addBurst(self):
        """Create one short-lived burst emitter (multiple emitters over time)."""
        emitter = ParticleEmitter("emitter_burst")
        emitter.setPos(
            random.uniform(-20, 20),
            45 + random.uniform(-5, 15),
            random.uniform(5, 25),
        )
        self.sphere.instanceTo(emitter)
        emitter.reparentTo(render)
        emitter.setColor(*random.choice(FIREWORK_COLORS))
        emitter.emitDuration = random.uniform(0.08, 0.2)
        emitter.tStop = emitter.tStart + emitter.emitDuration
        emitter.emitRate = random.uniform(600.0, 1200.0)
        emitter.emitVel = random.uniform(10.0, 18.0)
        emitter.emitAngle = random.uniform(150, 180)
        emitter.emitDir = Vec3(
            random.uniform(-0.15, 0.15),
            random.uniform(-0.15, 0.15),
            -1.0,
        ).normalized()
        emitter.emitLifetime = random.uniform(1.4, 2.2)
        self.particleEmitters.append(emitter)

    def spawnBurstEmitters(self, task):
        """Spawn new burst emitters over time."""
        t = globalClock.getRealTime()
        if t < self.nextBurstTime:
            return task.cont
        self.nextBurstTime = t + random.uniform(1.2, 2.5)
        self._addBurst()
        return task.cont

    def updateParticleEmitters(self, task):
        dt = globalClock.getDt()
        t = globalClock.getRealTime()
        if dt <= 0:
            return task.cont

        emittersToRemove = []

        for e in self.particleEmitters:
            if t >= e.tStop:
                emittersToRemove.append(e)
                continue
            # Emit only while within burst window
            e.emitAccum += e.emitRate * dt
            n = int(e.emitAccum)
            e.emitAccum -= n

            for _ in range(n):
                p = ParticleLifetime("p")
                p.setPos(e.getPos(render))
                p.tStart = globalClock.getRealTime()
                p.tStop = p.tStart + e.emitLifetime
                p.initialScale = random.uniform(0.05, 0.12)
                # Particle color based on emitter (faint)
                c = e.getColor()
                p.color = (c[0], c[1], c[2])

                dir_vec = randomInCone(e.emitAngle, e.emitDir)
                # Random speed multiplier to vary trajectories
                speed = e.emitVel * random.uniform(0.65, 1.35)
                p.vel = e.vel + dir_vec * speed
                # Small random jitter
                p.vel += Vec3(
                    random.uniform(-0.8, 0.8),
                    random.uniform(-0.8, 0.8),
                    random.uniform(-0.8, 0.8),
                )

                p.reparentTo(render)
                p.setScale(p.initialScale)
                p.setColor(p.color[0], p.color[1], p.color[2], 1.0)
                p.setTransparency(TransparencyAttrib.MAlpha)
                self.sphere.instanceTo(p)
                self.particles.append(p)

        # Destroy emitters when done (ParticleLifetime)
        for e in emittersToRemove:
            self.particleEmitters.remove(e)
            e.removeNode()

        return task.cont

    def updateParticles(self, task):
        dt = globalClock.getDt()
        t = globalClock.getRealTime()
        pRemove = []

        for p in self.particles:
            if t > p.tStop:
                pRemove.append(p)
                continue
            # Gravity
            a = Vec3(0.0, 0.0, -self.g)
            p.vel = p.vel + a * dt
            pos = p.getPos() + p.vel * dt
            p.setPos(pos)

            # Change particle colour depending on age (bright -> cooled, then fade)
            life = p.tStop - p.tStart
            age = t - p.tStart
            if life > 0:
                u = age / life  # 0 at birth, 1 at death
                # Lerp from emitter color to cooled (orange/dark) over first 60% of life
                blend = min(1.0, u / 0.6)
                cr = p.color[0] * (1 - blend) + COOLED_COLOR[0] * blend
                cg = p.color[1] * (1 - blend) + COOLED_COLOR[1] * blend
                cb = p.color[2] * (1 - blend) + COOLED_COLOR[2] * blend
                # Alpha: full for first half, then fade out
                alpha = 1.0 if u < 0.5 else max(0.0, 2.0 * (1.0 - u))
                p.setColor(cr, cg, cb, alpha)
                # Shrink toward end of life
                scaleF = 1.0 if u < 0.6 else max(0.03, (1.0 - u) / 0.4)
                p.setScale(p.initialScale * scaleF)

        for p in pRemove:
            self.particles.remove(p)
            p.removeNode()

        return task.cont


scene = SimpleScene()
scene.run()