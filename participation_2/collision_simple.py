# two spheres that bounce when they collide
import os
from direct.showbase.ShowBase import ShowBase
from panda3d.core import *

_ASSETS = os.path.dirname(os.path.abspath(__file__))

class Particle(NodePath):
    def __init__(self, *args, **kwargs):
        NodePath.__init__(self, *args, **kwargs)
        self.vel = Vec3(0.,0.,0.)
        self.inverseMass = 1.0
        self.radius = 1.0
   
class SimpleScene(ShowBase):
    def __init__(self):
        super().__init__(self)

        self.g = 0.0
        self.eCoeffRestitution = 1.0

        if (True):
            dlight = DirectionalLight('dlight')
            dlnp = render.attachNewNode(dlight)
            dlnp.setPos(20,0,40)
            dlnp.lookAt(0,0,0)
            render.setLight(dlnp)

        self.sphere = loader.loadModel(os.path.join(_ASSETS, "sphere.egg.pz"))
     
        self.particles = []
        for i in range(2):
            p = Particle("p")  
            self.sphere.instanceTo(p)
            p.setPos(Vec3((i-0.5)*5.3,25,i-0.5))
            p.vel = Vec3(1-i*2.,0., 0.)
            p.setScale(p.radius)
            p.reparentTo(render)
            self.particles.append(p)

        self.gameTask = taskMgr.add(self.updateParticles, "updateParticles")

    def updateParticles(self, task):
        dt = 1/60.

        for p in self.particles:
            p.vel = p.vel + Vec3(0.,0.,-self.g)*dt  
            p.setPos(p.getPos() + p.vel * dt)

        # collision between pairs
        for i,pi in enumerate(self.particles):
            for pj in self.particles[i+1:]:
                rij = pj.getPos() - pi.getPos()
                dist = rij.length()
                sumRad = pi.radius + pj.radius

                if dist < sumRad and dist > 0.0001:
                    n = rij / dist
                    vrel = (pi.vel - pj.vel).dot(n)
                    if vrel > 0:  # approaching
                        overlap = sumRad - dist
                        pi.setPos(pi.getPos() - n * overlap * 0.5)
                        pj.setPos(pj.getPos() + n * overlap * 0.5)
                        j = -(1 + self.eCoeffRestitution) * vrel / (pi.inverseMass + pj.inverseMass)
                        pi.vel += n * j * pi.inverseMass
                        pj.vel -= n * j * pj.inverseMass

        return task.cont
    
scene=SimpleScene()
scene.run()
