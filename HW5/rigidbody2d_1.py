from direct.showbase.ShowBase import ShowBase
from panda3d.core import *

class Particle(NodePath):
    def __init__(self, *args, **kwargs):
        NodePath.__init__(self, *args, **kwargs)
        self.vel = Vec3(0.,0.,0.)
        self.inverseMass = 1.0
        self.radius = 1.0

class RigidBody2D(Particle):
    def __init__(self, *args, **kwargs):
        Particle.__init__(self, *args, **kwargs)
        self.theta = 0.0
        self.omega = 0.0
        self.inverseMomentOfInertia = self.inverseMass/(2./3.*self.radius**2)
   
class SimpleScene(ShowBase):
    def __init__(self):
        super().__init__(self)

        self.g = 0.0
        self.eCoeffRestitution = 1.0
        self.dt = 1/60.

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
        for i in range(1):
            p = RigidBody2D("p")  
            self.cube.instanceTo(p)

            #Adjust physical and rendering properties of RigidBody2D here
            p.setPos(Vec3(0.,25.,0.)) #Middle of view
            p.vel = Vec3(0.,0., 0.)   #Not moving

            p.setScale(p.radius)
            p.setColor(Vec3(1,0,0))  #red
            p.reparentTo(render)
            self.RigidBody2Ds.append(p)

        self.gameTask = taskMgr.add(self.updateRigidBody2Ds, "updateRigidBody2Ds")

    def updateRigidBody2Ds(self, task):
        #dt = globalClock.getDt()  
        #use precise timesteps
        dt = self.dt

        for p in self.RigidBody2Ds:
            F = Vec3(0.1,0.,0.)  # simple a here
            rp = Vec3(0.,0.,0.25)
            
            vold = p.vel
            p.vel += F*p.inverseMass*dt  
            rold = p.getPos() 
            r = rold + p.vel * dt
            p.setPos(r)

            tau = rp.cross(F)
            p.omega += tau.y * p.inverseMomentOfInertia * dt
            p.theta += p.omega * dt
            p.setHpr( 0., 0., p.theta*180/3.141592653589793238462  )
        return task.cont
    
scene=SimpleScene()
scene.run()