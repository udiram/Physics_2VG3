from direct.showbase.ShowBase import ShowBase
from panda3d.core import *
import random

class Particle(NodePath):
    def __init__(self, *args, **kwargs):
        NodePath.__init__(self, *args, **kwargs)
        self.vel = Vec3(0.,0.,0.)
        self.inverseMass = 1.0

class ParticleLifetime(Particle):
    def __init__(self, *args, **kwargs):
        Particle.__init__(self, *args, **kwargs)
        self.tStart = globalClock.getRealTime()
        self.tStop = 1e37 #infinite

class ParticleEmitter(ParticleLifetime):
    def __init__(self, *args, **kwargs):
        ParticleLifetime.__init__(self, *args, **kwargs)
        self.emitAngle = 180 #half-angle of cone around emitting direction
        self.emitRate = 30.0 # per second
        self.emitVel = 10.0 # speed
        self.emitLifetime = 2.0 # seconds
 
class SimpleScene(ShowBase):
    def __init__(self):
        super().__init__(self)

        self.g = 9.8
        self.sphere = loader.loadModel("sphere.egg.pz")

        self.particles = []
        
        #Initialize particle emitter
        self.particleEmitters = []
        emitter = ParticleEmitter("emitter")
        emitter.setPos(0,50,-20)
        emitter.vel = Vec3(0,0,25.)
        self.sphere.instanceTo(emitter)
        emitter.reparentTo(render)
        emitter.setColor(1,0,0)
        emitter.setScale(0.2)
        self.particleEmitters.append(emitter)
        self.particles.append(emitter)

        self.gameTask = taskMgr.add(self.updateParticles, "updateParticles")
        self.emitTaks = taskMgr.add(self.updateParticleEmitters, "updateEmitters")

    def updateParticleEmitters(self, task):
        # Get the time elapsed since the next frame.  
        dt = globalClock.getDt()

        for e in self.particleEmitters:
            if (random.uniform(0,1) < e.emitRate*dt):
                p = ParticleLifetime("p")       

                p.setPos(e.getPos())
                while (True):
                    v = Vec3(random.uniform(-1,1),random.uniform(-1,1),random.uniform(-1,1))
                    if (v.length() < 1): break

                p.vel = e.vel + v.normalized()*e.emitVel
                p.tStop = p.tStart + e.emitLifetime

                #print("Making particle!",v, len(self.particles),p.tStop,globalClock.getRealTime())
                p.reparentTo(render)
                p.setScale(0.1)
                self.sphere.instanceTo(p)
                self.particles.append(p)

        return task.cont

    def updateParticles(self, task):
        # Get the time elapsed since the next frame.  
        dt = globalClock.getDt()
        t = globalClock.getRealTime()
        pRemove = []

        for p in self.particles:
            if (t > p.tStop):
                pRemove.append(p)
            else:
                a = Vec3(0.,0.,-self.g)  # simple a here
                p.vel = p.vel + a*dt  
                r = p.getPos() + p.vel * dt
                p.setPos(r)

        for p in pRemove:
            if hasattr(p,'emitRate'):
                self.particleEmitters.remove(p)
            self.particles.remove(p)   #remove from Python 
            p.removeNode()   #remove from Panda3d Scene -- no refs, garbage collect

        return task.cont
    
scene=SimpleScene()
scene.run()