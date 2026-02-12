import os
from direct.showbase.ShowBase import ShowBase
from panda3d.core import *

_ASSETS = os.path.dirname(os.path.abspath(__file__))

#colliding particle -- has bounding sphere
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
     
        #Initialize particles
        self.particles = []
        for i in range(2):
            p = Particle("p")  
            self.sphere.instanceTo(p)

            #Adjust physical and rendering properties of particle here
            p.setPos(Vec3((i-0.5)*5.3,25,i-0.5))
            p.vel = Vec3(1-i*2.,0., 0.)
            p.setScale(p.radius)
            p.reparentTo(render)
            self.particles.append(p)

        self.gameTask = taskMgr.add(self.updateParticles, "updateParticles")

    def updateParticles(self, task):
        # Get the time elapsed since the next frame. 
 
        #dt = globalClock.getDt()  
        #use precise timesteps
        dt = 1/60.

        for p in self.particles:
            a = Vec3(0.,0.,-self.g)  # simple a here
            vold = p.vel
            p.vel = p.vel + a*dt  
            rold = p.getPos() 
            r = rold + p.vel * dt
            p.setPos(r)

        #check other particles in pairs (exclude self)
        for i,pi in enumerate(self.particles):
            for pj in self.particles[i+1:]:
                #collision detection

                #check 1: overlap
                if (True):
                    #print("Overlap detected")

                    #check 2: must be approaching
                    if (True):
                        #print("Collision detected!")

                        #resolve collision         
                        pass
         

        return task.cont
    
scene=SimpleScene()
scene.run()