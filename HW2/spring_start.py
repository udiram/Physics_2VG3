import math
import sys
from direct.showbase.ShowBase import ShowBase
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import *

class Particle(NodePath):
    def __init__(self, *args, **kwargs):
        NodePath.__init__(self, *args, **kwargs)
        self.vel = Vec3(0., 0., 0.)
        self.inverseMass = 1.0
        self.force = Vec3(0., 0., 0.)


class SimpleScene(ShowBase):
    def __init__(self):
        super().__init__(self)

        # light setup
        dlight = DirectionalLight('dlight')
        dlnp = render.attachNewNode(dlight)
        dlnp.setPos(50, 0, 100)
        dlnp.lookAt(0, 50, 0)
        render.setLight(dlnp)

        self.springLength = 15.
        self.springK = 6000.0
        self.kIncrement = 1.2  # ramp k up until things blow up
        self.frameCount = 0
        self.lastGoodK = self.springK
        self.lastGoodFrame = 0
        self.sphere = loader.loadModel("sphere.egg.pz")

        self.kLabel = OnscreenText(text=f"k = {self.springK:.4g}", pos=(-1.25, 0.92),
                                   scale=0.05, fg=(1, 1, 1, 1), align=TextNode.ALeft)

        # init the two spheres
        self.particles = []
        for i in range(2):
            p = Particle("p")  
            self.sphere.instanceTo(p)
            p.setColor(1.0 if i == 0 else 0.0, 1.0 if i == 1 else 0.0, 1.0 if i == 2 else 0.0, 1.0)
            p.setPos(Vec3((i - 0.5) * 20., 100., (i - 0.5) * 10.))
            p.vel = Vec3(0., 0., 0.)
            p.reparentTo(render)
            self.particles.append(p)

        self.gameTask = taskMgr.add(self.updateParticles, "updateParticles")

    def updateParticles(self, task):
        dt = globalClock.getDt()
        if dt > 0.02:
            dt = 0.02

        # keep bumping k until we hit numerical instability
        self.springK *= self.kIncrement ** dt
        self.kLabel.setText(f"k = {self.springK:.4g}")

        def vec_isfinite(v):
            return math.isfinite(v.getX()) and math.isfinite(v.getY()) and math.isfinite(v.getZ())

        # spring between p0 and p1
        r01 = self.particles[1].getPos()-self.particles[0].getPos()
        if not vec_isfinite(r01):
            print(f"Blowup at r01: frame={self.frameCount}, k={self.springK:.4g}, r01={r01}")
            print(f"  last good: frame={self.lastGoodFrame}, k={self.lastGoodK:.4g}")
            sys.exit(0)
        r01_len = r01.length()
        if r01_len == 0:
            print(f"Blowup at r01.normalized() (zero length): frame={self.frameCount}, k={self.springK:.4g}")
            print(f"  last good: frame={self.lastGoodFrame}, k={self.lastGoodK:.4g}")
            sys.exit(0)
        spring_force = r01.normalized() * (r01_len - self.springLength) * self.springK
        if not vec_isfinite(spring_force):
            print(f"Blowup at spring_force: frame={self.frameCount}, k={self.springK:.4g}, force={spring_force}")
            print(f"  last good: frame={self.lastGoodFrame}, k={self.lastGoodK:.4g}")
            sys.exit(0)
        self.particles[0].force = spring_force
        self.particles[1].force = -spring_force

        for i, p in enumerate(self.particles):
            a = p.force * p.inverseMass
            if not vec_isfinite(a):
                print(f"Blowup at acceleration (particle {i}): frame={self.frameCount}, k={self.springK:.4g}, a={a}, force={p.force}")
                print(f"  last good: frame={self.lastGoodFrame}, k={self.lastGoodK:.4g}")
                sys.exit(0)
            p.vel = p.vel + a * dt
            if not vec_isfinite(p.vel):
                print(f"Blowup at velocity (particle {i}): frame={self.frameCount}, k={self.springK:.4g}, vel={p.vel}")
                print(f"  last good: frame={self.lastGoodFrame}, k={self.lastGoodK:.4g}")
                sys.exit(0)
            r = p.getPos() + p.vel * dt
            if not vec_isfinite(r):
                print(f"Blowup at position (particle {i}): frame={self.frameCount}, k={self.springK:.4g}, r={r}, vel={p.vel}, pos={p.getPos()}")
                print(f"  last good: frame={self.lastGoodFrame}, k={self.lastGoodK:.4g}")
                sys.exit(0)
            p.setPos(r)

        # shouldn't get here if the above checks work
        if not all(vec_isfinite(p.getPos()) and vec_isfinite(p.vel) for p in self.particles):
            print(f"Blowup: last rendered frame = {self.lastGoodFrame}, k = {self.lastGoodK:.4g}")
            sys.exit(0)

        self.lastGoodK = self.springK
        self.lastGoodFrame = self.frameCount
        self.frameCount += 1
        return task.cont


scene = SimpleScene()
scene.run()