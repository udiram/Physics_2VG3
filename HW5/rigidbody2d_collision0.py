from direct.showbase.ShowBase import ShowBase
from panda3d.core import *

class Particle(NodePath):
    def __init__(self, *args, **kwargs):
        NodePath.__init__(self, *args, **kwargs)
        self.vel = Vec3(0.,0.,0.)
        self.inverseMass = 1.0
        self.radius = 1.0
        self.collisionRadius = self.radius

class RigidBody2D(Particle):
    def __init__(self, *args, **kwargs):
        Particle.__init__(self, *args, **kwargs)
        self.theta = 0.0
        self.omega = 0.0
        #self.inverseMomentOfInertia = 
        #self.collisionRadius = 
   
class SimpleScene(ShowBase):
    def __init__(self):
        super().__init__(self)

        self.g = 0.0
        self.eCoeffRestitution = 1.0

        if (True):
            dlight = DirectionalLight('dlight')
            dlnp = render.attachNewNode(dlight)
            dlnp.setPos(10,0,10)
            dlnp.lookAt(0,25,0)
            render.setLight(dlnp)

        self.cube = loader.loadModel("Cube")
        self.cube.setScale(0.6)  
     
        #Initialize RigidBody2Ds
        self.RigidBody2Ds = []
        for i in range(2):
            p = RigidBody2D("p")  
            self.cube.instanceTo(p)

            #Adjust physical and rendering properties of particle here
            p.setPos(Vec3(-6+17.*i,25,i-0.5))
            p.vel = Vec3(2-i*6.,0., 0.)
            p.setScale(p.radius)
            p.setColor(Vec3(i,1-i,0))
            p.reparentTo(render)
            self.RigidBody2Ds.append(p)

        self.gameTask = taskMgr.add(self.updateRigidBody2Ds, "updateRigidBody2Ds")

    def updateRigidBody2Ds(self, task):
        # Get the time elapsed since the next frame. 
 
        #dt = globalClock.getDt()  
        #use precise timesteps
        dt = 1/60.

        for p in self.RigidBody2Ds:
            a = Vec3(0.,0.,-self.g)  # simple a here
            vold = p.vel
            p.vel = p.vel + a*dt  
            rold = p.getPos() 
            r = rold + p.vel * dt
            p.setPos(r)

            #Update angular info

        #check other RigidBody2Ds in pairs (exclude self)
        for i,pi in enumerate(self.RigidBody2Ds):
            for pj in self.RigidBody2Ds[i+1:]:
                #collision detection
                xi = pi.getPos()
                xj = pj.getPos()

                rij = xj-xi
                #check 1: possible overlap
                if (True):
                    print("Possible contact")

                    #check 2: find collision point (if one exists)
                    if (True):
                        print("Contact found")

                        #check 3: must be approaching
                        if (True):
                            print("Collision detected!")

                            #resolve collision

        return task.cont
    
scene=SimpleScene()
scene.run()