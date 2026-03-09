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
        self.inverseMomentOfInertia = self.inverseMass/(2./3.*self.radius**2)
        self.collisionRadius = 1.8*self.radius
   
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

    def collideSquareSquare( self, x1, x2, radius1, radius2, angle1, angle2 ):
        square = [ Vec3(-1,0,-1), Vec3(-1,0,1), Vec3(1,0,1), Vec3(1,0,-1) ]

        m1 = Mat3().rotateMatNormaxis( angle1*180/3.14159265, Vec3(0,1,0) )
        m2 = Mat3().rotateMatNormaxis( angle2*180/3.14159265, Vec3(0,1,0) )

        square1=[]
        square2=[]

        for vertex in square:
            square1.append( x1 + m1.xform( vertex*radius1 ) )
            square2.append( x2 + m2.xform( vertex*radius2 ) )

        return (None,None,None)

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

            p.theta += p.omega * dt
            p.setHpr( 0., 0., p.theta*180/3.141592653589793238462  )

        #check other RigidBody2Ds in pairs (exclude self)
        for i,pi in enumerate(self.RigidBody2Ds):
            for pj in self.RigidBody2Ds[i+1:]:
                #collision detection
                xi = pi.getPos()
                xj = pj.getPos()

                rij = xj-xi
                #check 1: possible overlap
                if (rij.length() < pi.collisionRadius + pj.collisionRadius):
                    print("Possible contact")

                    #check 2: find collision point (if one exists)
                    # d,xclose,normal = self.collideSquareSquare( xi, xj, pi.radius, pj.radius, pi.theta, pj.theta )
                    if (abs(rij.x) < pi.radius + pj.radius): #hack -- assumes face-face collision normal in x-direction
                        print("Contact found")

                        ui = pi.vel
                        uj = pj.vel
                        uij = uj-ui
                        #check 3: must be approaching
                        if (uij.dot(rij) < 0):  #Hack -- should use normal and full velocity with omega
                            print("Collision detected!")

                            #resolve collision
                            nij = Vec3(1,0,0)  #hack: force normal to be in x-direction
                            rp = (xi+xj)*0.5   #hack: assume collision location
                            rip = rp-xi
                            rjp = rp-xj
                            ripcn = rip.cross(nij)
                            rjpcn = rjp.cross(nij)
                            dp = nij*(1+self.eCoeffRestitution)/ \
                                (pi.inverseMass+pj.inverseMass+ \
                                 ripcn.dot(ripcn)*pi.inverseMomentOfInertia + \
                                      rjpcn.dot(rjpcn)*pj.inverseMomentOfInertia)*uij.dot(nij)        
                            pi.vel += dp*pi.inverseMass
                            pj.vel -= dp*pj.inverseMass 
                            pi.omega += (rip.cross(dp)).y*pi.inverseMomentOfInertia
                            pj.omega -= (rjp.cross(dp)).y*pj.inverseMomentOfInertia
                            print(dp)


        return task.cont
    
scene=SimpleScene()
scene.run()
