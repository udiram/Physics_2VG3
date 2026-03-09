from direct.showbase.ShowBase import ShowBase
from panda3d.core import *
from rigidbody2d_mod import *
from rigidbody2d_collision_mod import *

class SimpleScene(ShowBase):
    def __init__(self):
        super().__init__(self)

        #Use Orthographic projection (2D)
        lens = OrthographicLens()
        lens.setFilmSize(8.,6.)
        lens.setNearFar(-100,100)
        self.cam.node().setLens(lens)

        self.g = 10.0
        self.eCoeffRestitution = 0.2
        self.muFrictionStatic = 0.5
        self.muFrictionKinetic = 0.5

        self.dt = 1/60.
        self.t = 0.
        self.iFrame = 0
        self.fixPos = True  #Fix positions
        self.fixVel = False

        self.cube = loader.loadModel("Cube")
        self.cube.setScale(0.6)  

        markerInit(self,"samples/solar-system/models/planet_sphere")  #collision diagnostics
 
        self.RigidBody2Ds = []    
        self.scenario = 0
        self.nextScenario()
  
        self.gameTask = taskMgr.add(self.updateRigidBody2Ds, "updateRigidBody2Ds")

    def nextScenario(self):
        self.dt = 1/60

        for p in self.RigidBody2Ds: 
            p.removeNode() 

        #Initialize RigidBody2Ds
        self.RigidBody2Ds = []
        for i in range(2):
            p = RigidBody2D("p")
            p.doGravity = False
            p.inverseMass = 1.0
            self.cube.instanceTo(p)
   
            p.setScale(p.radius)
            p.setColor(Vec3(1,i,0))
            p.reparentTo(render)
            self.RigidBody2Ds.append(p)

        #scenarios
        print('scenario=%i e=%f' % (self.scenario,self.eCoeffRestitution))        
        if (self.scenario==0):
            #inclined plane -- add setup code here
            self.RigidBody2Ds[0].doGravity = True
 
    def updateRigidBody2Ds(self, task):
        markerClean(self)

        #use precise timesteps
        dt = self.dt
        self.t += self.dt
        self.iFrame += 1

        for p in self.RigidBody2Ds:
            a = Vec3(0.,0.,0.)  # simple a here
            #If putting to sleep do something here
            if (p.doGravity): a = Vec3(0.,0.,-self.g)  
            p.vold = p.vel
            p.vel = p.vel + a*dt  
            p.rold = p.getPos() 
            r = p.rold + p.vel * dt
            p.setPos(r)
            p.theta += p.omega * dt
            p.setHpr( 90., p.theta*180/3.141592653589793238462, 0. )

        #check other RigidBody2Ds in pairs (exclude self)
        for i,pi in enumerate(self.RigidBody2Ds):
            for pj in self.RigidBody2Ds[i+1:]:
                #collision detection
                xi = pi.getPos()
                xj = pj.getPos()

                rij = xj-xi
                #check 1: inside 3d sphere that encloses bodies
                if (abs(rij.length()) < pi.collisionRadius + pj.collisionRadius):
                    #print("Possible 3d collision")
                    #Find closest points of contact
                    contacts = collideSquareSquare( self, xi, xj, pi.radius, pj.radius, pi.theta, pj.theta )
                    ncontacts = 0
                    # Add variables to accumulate change due to all contacts
                    for contact in contacts:
                        dclose,xclose,normal = contact

                        ui = pi.vel + Vec3(0.,pi.omega,0.).cross(xclose-xi)
                        uj = pj.vel + Vec3(0.,pj.omega,0.).cross(xclose-xj)

                        uij = uj-ui
                        #check 3: must be approaching
                        if (uij.dot(normal) < 0.):
                            #print("Collision detected!")
                            ncontacts += 1
                            #resolve collision
                            nij = normal
                            dxi = (xclose-xi)
                            dxj = (xclose-xj)  
                            rni = dxi.cross(nij)
                            rnj = dxj.cross(nij)

                            iMFac = 1/(pi.inverseMass+pj.inverseMass + \
                            rni.dot(rni)*pi.inverseMomentOfInertia + rnj.dot(rnj)*pj.inverseMomentOfInertia)
                            dp = nij*(1+self.eCoeffRestitution)*uij.dot(nij)*iMFac

                            #old code - apply full effect of each contact
                            pi.vel += dp*pi.inverseMass
                            pj.vel -= dp*pj.inverseMass 
                            pi.omega += (dxi.cross(dp)).y*pi.inverseMomentOfInertia
                            pj.omega -= (dxj.cross(dp)).y*pj.inverseMomentOfInertia

                            # Add code to calculate impulse parallel to surface

                            # Add code to fix position of collider so it doesn't slip into surface
                            #if (self.fixPos):

                            #Add effect of this contact to accumulator variables

                    #add accumulated change to rigid bodies
                    #Divide total effect by number of contacts
                    #if (ncontacts > 0):
                        #if (self.fixPos):
                        
        
        return task.cont
    
scene=SimpleScene()
scene.run()