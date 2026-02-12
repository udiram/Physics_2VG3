import os
from direct.showbase.ShowBase import ShowBase
from panda3d.core import *

_ASSETS = os.path.dirname(os.path.abspath(__file__))

class Particle(NodePath):
    def __init__(self, *args, **kwargs):
        NodePath.__init__(self, *args, **kwargs)
        self.vel = Vec3(0.,0.,0.)
        self.inverseMass = 1.0

   
class SimpleScene(ShowBase):
    def __init__(self):
        super().__init__(self)

        self.g = 9.8
        self.sphere = loader.loadModel(os.path.join(_ASSETS, "sphere.egg.pz"))
        self.sphereRadius = 1.0   #matches sphere model
        self.box = loader.loadModel(os.path.join(_ASSETS, "box.egg"))
        self.box.setScale(100)
        self.box.setPos(-50.,50.,-100)
        self.box.reparentTo(render)
        
        #Initialize particles
        self.particles = []
        for i in range(3):
            p = Particle("p")  
            self.sphere.instanceTo(p)

            p.setPos(Vec3((i-1)*5,50,self.sphereRadius))
            p.vel = Vec3(0.,0., i*10.0)
            p.reparentTo(render)
            self.particles.append(p)

        self.gameTask = taskMgr.add(self.updateParticles, "updateParticles")

    def updateParticles(self, task):
        # Get the time elapsed since the next frame.  
        dt = globalClock.getDt()

        for p in self.particles:
            a = Vec3(0.,0.,-self.g)  # simple a here
            p.vel = p.vel + a*dt  
            rold = p.getPos() 
            r = rold + p.vel * dt

            if (r.z < self.sphereRadius):
                p.vel.z = abs(p.vel.z)
            p.setPos(r)

        return task.cont
    
scene=SimpleScene()
scene.run()
