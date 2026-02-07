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

        self.springLength = 17.
        self.springK = 10.0
        hw2_dir = os.path.dirname(os.path.abspath(__file__))
        self.sphere = loader.loadModel(os.path.join(hw2_dir, "sphere.egg.pz"))
        self.cylinderModel = loader.loadModel(os.path.join(hw2_dir, "Cylinder.egg"))
        self.cylinderModel.setScale(Vec3(1,1,10.))
        self.cylinderModel.setHpr(0,90,0)
        self.cylinderContainer = NodePath("cylinderContainer")
        self.cylinderContainer.setColor(1,1,1)
        self.cylinderContainer.setPos(0,100,0)
        self.cylinderContainer.reparentTo(render)
        self.cylinderModel.reparentTo(self.cylinderContainer)

        #Initialize particles
        self.particles = []
        for i in range(2):
            p = Particle("p")  
            self.sphere.instanceTo(p)
            p.setColor( 1.0 if i==0 else 0.0, 1.0 if i==1 else 0.0, 1.0 if i==2 else 0.0, 1.0)

            p.setPos(Vec3((i-0.5)*20.-20.,100.,(i-0.5)*10.))
            p.vel = Vec3(2.,0.,(i-0.5)*5.)
            p.reparentTo(render)
            self.particles.append(p)

        self.gameTask = taskMgr.add(self.updateParticles, "updateParticles")

    def updateParticles(self, task):
        # Get the time elapsed since the next frame.  
        dt = globalClock.getDt()
        dt=1/60.

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

        #Stretch cylinder to roughly extend from one mass to the other
        self.cylinderContainer.setPos((self.particles[0].getPos()+self.particles[1].getPos())*0.5)
        self.cylinderContainer.lookAt(self.particles[0].getPos())
        self.cylinderModel.setScale(0.1,0.1,r01.length()*0.275)

        return task.cont
    
scene=SimpleScene()
scene.run()