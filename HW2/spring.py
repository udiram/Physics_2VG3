from direct.showbase.ShowBase import ShowBase
from panda3d.core import *
import os

class Particle(NodePath):
    def __init__(self, *args, **kwargs):
        NodePath.__init__(self, *args, **kwargs)
        self.vel = Vec3(0.,0.,0.)
        self.inverseMass = 1.0
        self.force = Vec3(0.,0.,0.)

   
class SimpleScene(ShowBase):
    def __init__(self):
        super().__init__(self)

        dlight = DirectionalLight('dlight')
        dlnp = render.attachNewNode(dlight)           
        dlnp.setPos(50,0,100)
        dlnp.lookAt(0,50,0)
        render.setLight(dlnp)

        self.springLength = 15.
        self.springK = 1.0
        hw2_dir = os.path.dirname(os.path.abspath(__file__))
        self.sphere = loader.loadModel(os.path.join(hw2_dir, "sphere.egg.pz"))

        #Initialize particles
        self.particles = []
        for i in range(2):
            p = Particle("p")  
            self.sphere.instanceTo(p)
            p.setColor( 1.0 if i==0 else 0.0, 1.0 if i==1 else 0.0, 1.0 if i==2 else 0.0, 1.0)

            p.setPos(Vec3((i-0.5)*20.,100.,(i-0.5)*10.))
            p.vel = Vec3(0.,0.,0.)
            p.reparentTo(render)
            self.particles.append(p)

        self.gameTask = taskMgr.add(self.updateParticles, "updateParticles")

    def updateParticles(self, task):
        # Get the time elapsed since the next frame.  
        dt = globalClock.getDt()
        if (dt > 0.02): dt=0.02

        # Hardcoded to put spring between first two particles
        r01 = self.particles[1].getPos()-self.particles[0].getPos()
        SpringForce = r01.normalized()*(r01.length()-self.springLength)*self.springK
        self.particles[0].force = SpringForce
        self.particles[1].force = -SpringForce
              
        for p in self.particles:
            a = p.force*p.inverseMass
            p.vel = p.vel + a*dt  
            r = p.getPos() + p.vel * dt
            p.setPos(r)

        return task.cont
    
scene=SimpleScene()
scene.run()