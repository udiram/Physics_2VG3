from direct.showbase.ShowBase import ShowBase
from panda3d.core import *
import random
import math

class Particle(NodePath):
    def __init__(self, *args, **kwargs):
        NodePath.__init__(self, *args, **kwargs)
        self.vel = Vec3(0., 0., 0.)
        self.inverseMass = 1.0

class ParticleLifetime(Particle):
    def __init__(self, *args, **kwargs):
        Particle.__init__(self, *args, **kwargs)
        self.tStart = globalClock.getRealTime()
        self.tStop = 1e37  # infinite by default

class ParticleEmitter(ParticleLifetime):
    def __init__(self, *args, **kwargs):
        ParticleLifetime.__init__(self, *args, **kwargs)
        self.emitAngle = 30           # half-angle cone (degrees)
        self.emitRate = 200.0         # per second (dense burst)
        self.emitVel = 18.0           # particle speed
        self.emitLifetime = 2.0       # particle lifetime
        self.emissionDuration = 0.9   # seconds emitter actively emits
        self.baseColor = Vec4(1, 1, 1, 1)

        self.nextEmitTime = self.tStart  # for scheduled burst emission

class SimpleScene(ShowBase):
    def __init__(self):
        super().__init__(self)

        self.g = 9.8
        self.sphere = loader.loadModel("./panda3d/sphere.egg.pz")

        # Enable alpha blending so particles can be faint/fade
        render.setTransparency(TransparencyAttrib.MAlpha)

        self.particles = []
        self.particleEmitters = []

        # Make multiple emitters (multiple fireworks)
        for _ in range(4):
            self.spawn_emitter()

        self.gameTask = taskMgr.add(self.updateParticles, "updateParticles")
        self.emitTask = taskMgr.add(self.updateParticleEmitters, "updateEmitters")

    def spawn_emitter(self):
        e = ParticleEmitter("emitter")

        # Random launch position and upward velocity
        e.setPos(random.uniform(-10, 10), 50, random.uniform(-25, -15))
        e.vel = Vec3(0, 0, random.uniform(22, 30))

        # Random style parameters
        e.emitAngle = random.uniform(15, 45)
        e.emitVel = random.uniform(12, 22)
        e.emitRate = random.uniform(150, 350)  # dense
        e.emitLifetime = random.uniform(1.3, 2.3)
        e.emissionDuration = random.uniform(0.5, 1.2)

        # Give each emitter a base color (faint, slightly transparent)
        palette = [
            Vec4(1.0, 0.2, 0.2, 0.9),   # red
            Vec4(0.2, 0.6, 1.0, 0.9),   # blue
            Vec4(0.2, 1.0, 0.4, 0.9),   # green
            Vec4(1.0, 0.9, 0.2, 0.9),   # yellow
            Vec4(1.0, 0.4, 1.0, 0.9),   # magenta
        ]
        e.baseColor = random.choice(palette)

        # Emitter lifetime: it exists long enough to emit, then gets destroyed
        now = globalClock.getRealTime()
        e.tStart = now
        e.tStop = now + e.emissionDuration  # after this, stop emitting and remove emitter

        self.sphere.instanceTo(e)
        e.reparentTo(render)
        e.setColor(e.baseColor)
        e.setScale(0.25)

        self.particleEmitters.append(e)
        self.particles.append(e)

    def updateParticleEmitters(self, task):
        dt = globalClock.getDt()
        now = globalClock.getRealTime()

        # You can continuously spawn new fireworks when old ones end:
        if len(self.particleEmitters) < 4:
            if random.uniform(0, 1) < 0.02:  # small chance per frame
                self.spawn_emitter()

        for e in list(self.particleEmitters):
            # Only emit while emitter is "alive"
            if now > e.tStop:
                continue

            # Dense emission: do expected number this frame (stochastic)
            expected = e.emitRate * dt
            count = int(expected)
            if random.uniform(0, 1) < (expected - count):
                count += 1

            for _ in range(count):
                p = ParticleLifetime("p")
                p.setPos(e.getPos())

                # Random direction with upward bias
                v = Vec3(
                    random.uniform(-1, 1),
                    random.uniform(-1, 1),
                    random.uniform(0.2, 1.0)  # upward bias
                )

                v.normalize()

                # Speed variation
                speed = e.emitVel * random.uniform(0.8, 1.2)

                # Particle velocity = emitter velocity + explosion velocity
                p.vel = e.vel + v * speed

                # Particle lifetime
                p.tStart = now
                p.tStop = now + e.emitLifetime * random.uniform(0.7, 1.1)

                # Color inherited from emitter
                p.baseColor = Vec4(e.baseColor)

                p.reparentTo(render)
                self.sphere.instanceTo(p)
                p.setScale(random.uniform(0.06, 0.12))

                # Start slightly faint
                c = Vec4(p.baseColor)
                c.w = 0.7
                p.setColor(c)

                self.particles.append(p)

        return task.cont

    def updateParticles(self, task):
        dt = globalClock.getDt()
        now = globalClock.getRealTime()
        pRemove = []

        for p in self.particles:
            if now > p.tStop:
                pRemove.append(p)
                continue

            # Physics: gravity + slight drag to look less ballistic
            a = Vec3(0., 0., -self.g)
            p.vel += a * dt

            drag = 0.15  # tweak for look
            p.vel *= (1.0 - drag * dt)

            p.setPos(p.getPos() + p.vel * dt)

            # --- Age-based color/alpha fade for particles ---
            if hasattr(p, "tStart") and not hasattr(p, "emitRate"):  # it's a particle, not emitter
                age = now - p.tStart
                life = max(1e-6, p.tStop - p.tStart)
                u = max(0.0, min(1.0, age / life))  # normalized age

                # Bright early, dim late, fade alpha too
                base = Vec4(p.baseColor)
                brightness = (1.0 - u) ** 0.5
                alpha = 0.8 * (1.0 - u)

                # slight shift toward white early (spark), then toward base color
                c = Vec4(
                    min(1.0, base.x * (0.6 + 0.4*brightness) + 0.4*brightness),
                    min(1.0, base.y * (0.6 + 0.4*brightness) + 0.4*brightness),
                    min(1.0, base.z * (0.6 + 0.4*brightness) + 0.4*brightness),
                    max(0.0, alpha)
                )
                p.setColor(c)

        for p in pRemove:
            if hasattr(p, 'emitRate'):
                # emitter cleanup
                if p in self.particleEmitters:
                    self.particleEmitters.remove(p)
            if p in self.particles:
                self.particles.remove(p)
            p.removeNode()

        return task.cont

scene = SimpleScene()
scene.run()