from direct.showbase.ShowBase import ShowBase
from panda3d.core import *
from direct.gui.OnscreenText import OnscreenText
import math #NEW
from math import cos, sin #NEW
import copy #NEW
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
        self.eCoeffRestitution = 0.2 #NEW 
        self.muFrictionStatic = 0.5
        self.muFrictionKinetic = 0.5

        self.dt = 1/60.  
        self.t = 0.
        self.iFrame = 0
        self.fixPos = True  #Fix positions
        self.fixVel = False

        self.cube = loader.loadModel("./panda3d/Cube.egg")
        self.cube.setScale(0.6)  

        self.tube = loader.loadModel("./panda3d/Tube.egg")  #GONE: NOT USED
        self.tube.setScale(1.3/1.41421356237)
        self.tube.setHpr(90.,90.,0.)
        self.tube.setPos(0,0.1,0)

        markerInit(self,"./panda3d/sphere.egg.pz")  #collision diagnostics

        self.RigidBody2Ds = []    
        self.scenario = 0 #NEW 0 or 1
        self.nextScenario()
  
        self.gameTask = taskMgr.add(self.updateRigidBody2Ds, "updateRigidBody2Ds")
   
    def nextScenario(self):
        self.dt = 1/60

        for p in self.RigidBody2Ds: 
            p.removeNode() 

        #Initialize RigidBody2Ds
        self.RigidBody2Ds = []
        for i in range(2+self.scenario*4):
            p = RigidBody2D("p")
            p.inverseMass = 1.0
            #self.cube.instanceTo(p)
   
            p.setScale(p.radius)
            p.setColor(Vec3(1-(i&2)//2,i&1,i&4))  #NEW
            p.reparentTo(render)
            self.RigidBody2Ds.append(p)

        #scenarios
        print('scenario=%i e=%f' % (self.scenario,self.eCoeffRestitution))
        
        if (self.scenario==0 or self.scenario==1): #NEW
            #inclined plane
            angle=0.10016742116  #NEW adjustable angle
            self.RigidBody2Ds[0].radius = 0.5
            self.RigidBody2Ds[0].setScale(0.5)
            self.RigidBody2Ds[0].setPos(-0.,3,self.RigidBody2Ds[0].radius)
            self.RigidBody2Ds[0].vel = Vec3(-sin(angle),0,-cos(angle))
            self.RigidBody2Ds[0].doGravity = True
            self.RigidBody2Ds[0].shape = "cylinder"  #REMOVE: square is default
            self.tube.instanceTo(self.RigidBody2Ds[0])  #REMOVE: back to the cube
            self.RigidBody2Ds[0].inverseMomentOfInertia = self.RigidBody2Ds[0].inverseMass/(2./3.*self.RigidBody2Ds[0].radius**2) 

            self.RigidBody2Ds[1].radius = 2.0
            self.RigidBody2Ds[1].setScale(2.0)
            self.RigidBody2Ds[1].setPos(0.2,3,-self.RigidBody2Ds[1].radius-1.)
            self.RigidBody2Ds[1].vel = Vec3(0,0,0)
            self.RigidBody2Ds[1].inverseMass = 0.0
            self.cube.instanceTo(self.RigidBody2Ds[1])
            self.RigidBody2Ds[1].inverseMomentOfInertia = 0.0

            #incline
            self.RigidBody2Ds[0].theta = angle
            self.RigidBody2Ds[1].theta = angle 

            if (len(self.RigidBody2Ds)>2): #NEW
                i=2
                for r in self.RigidBody2Ds[2:]:
                    r.radius = 0.3-i*0.01
                    r.setScale(0.3-i*0.01)
                    r.inverseMass = 1/0.2
                    self.cube.instanceTo(r)
                    r.inverseMomentOfInertia = r.inverseMass/(2./3.*r.radius**2)

                    r.setPos(-0.1+(i%3)*0.05,3,2.0+(i-2)*0.5)
                    r.vel = Vec3(-sin(angle),0,-cos(angle))
                    r.doGravity = True
                    r.theta =  angle
                    i += 1            
 
    def updateRigidBody2Ds(self, task):
        markerClean(self)

        #use precise timesteps
        dt = self.dt
        self.t += self.dt
        self.iFrame += 1

        for p in self.RigidBody2Ds:
            a = Vec3(0.,0.,0.)  # simple a here
            if (p.doGravity): a = Vec3(0.,0.,-self.g)  
            #p.vold = copy.deepcopy(p.vel) #NEW need a deepcopy or p.vold is just a pointer to p.vel!
            p.vel = p.vel + a*dt  
            p.rold = p.getPos() 
            r = p.rold + p.vel * dt
            p.setPos(r)            
            p.thetaold = p.theta  #NEW
            p.theta += p.omega * dt
            p.setHpr( 90., p.theta*180/3.141592653589793238462, 0. )

        #check other RigidBody2Ds in pairs (exclude self)
        #NEW: all this code was moved over one indent
        for i,pi in enumerate(self.RigidBody2Ds):
            for pj in self.RigidBody2Ds[i+1:]:
                #collision detection
                xi = pi.getPos()
                xj = pj.getPos()

                rij = xj-xi
                #check 1: inside 3d sphere that encloses bodies
                if (abs(rij.length()) < pi.collisionRadius + pj.collisionRadius):
                    #print("Possible collision")
                    #Find closest points of contact
                    #contacts = collideSquareSquare( self, xi, xj, pi.radius, pj.radius, pi.theta, pj.theta )
                    contacts = collideShapes( self, pi, pj )
                    
                    # Add variables to accumulate change due to all contacts  GONE
                    dpiPos = Vec3(0,0,0)
                    dpjPos = Vec3(0,0,0)
                    dpiVel = Vec3(0,0,0)
                    dpjVel = Vec3(0,0,0)
                    dpiOmega = 0.
                    dpjOmega = 0.

                    ncontacts = 0 #GONE
                    for contact in contacts:
                        dclose,xclose,normal = contact

                        ui = pi.vel + Vec3(0.,pi.omega,0.).cross(xclose-xi)
                        uj = pj.vel + Vec3(0.,pj.omega,0.).cross(xclose-xj)

                        uij = uj-ui
                        nij = normal #NEW
                        unormal = uij.dot(nij) #NEW
                        #check 3: must be approaching
                        if (uij.dot(normal) < 0.):  #GONE
                            #print("Collision detected!")
                            ncontacts += 1  #GONE

                            #First iteration
                            #if (iter==0): #NEW


                            #resolve collision
                            nij = normal
                            dxi = (xclose-xi)
                            dxj = (xclose-xj)  
                            rni = dxi.cross(nij)
                            rnj = dxj.cross(nij)

                            iMFac = 1/(pi.inverseMass+pj.inverseMass + \
                            rni.dot(rni)*pi.inverseMomentOfInertia + rnj.dot(rnj)*pj.inverseMomentOfInertia)
                            dp = nij*(1+self.eCoeffRestitution)*uij.dot(nij)*iMFac  #GONE
                            # NEW code for dpNormal


                            # Add code to calculate impulse parallel to surface
                            FricVecij = uij-nij*uij.dot(nij)
                            FricDir = FricVecij.normalized() #NEW
                               
                            #GONE
                            FricRatio = FricVecij.length()/(abs(uij.dot(nij)) + 1e-20) #Remove FricRatio here
                            if (FricRatio > self.muFrictionStatic):
                                FricRatio = self.muFrictionKinetic
                            dp += -FricDir*FricRatio*(1+self.eCoeffRestitution)*uij.dot(nij)*iMFac

                            #New code for dpFric    


                            # Add code to fix position of collider so it doesn't slip into surface
                            if (self.fixPos):  #GONE
                                #Stop the box creeping down the incline
                                dpiPos += FricDir*FricRatio*dclose*pi.inverseMass/(pi.inverseMass+pj.inverseMass)
                                dpjPos -= FricDir*FricRatio*dclose*pj.inverseMass/(pi.inverseMass+pj.inverseMass)

                            #Add effect of this contact to accumulator variables GONE
                            dpiVel += dp*pi.inverseMass #GONE
                            dpjVel -= dp*pj.inverseMass 
                            dpiOmega += (dxi.cross(dp)).y*pi.inverseMomentOfInertia
                            dpjOmega -= (dxj.cross(dp)).y*pj.inverseMomentOfInertia

                            #NEW update velocity and rotation each iteration                   
 

                            #NEW advance to next contact after all changes done  
  
                    #add accumulated change to rigid bodies
                    #Divide total effect by number of contacts
                    if (ncontacts > 0):  #GONE
                        pi.vel += dpiVel/ncontacts
                        pj.vel += dpjVel/ncontacts
                        pi.omega += dpiOmega/ncontacts
                        pj.omega += dpjOmega/ncontacts
                        if (self.fixPos):
                            pi.setPos(xi + nij*(pi.rold-xi).dot(nij) )#+dpiPos/ncontacts)
                            pj.setPos(xj + nij*(pj.rold-xj).dot(nij) )#+dpjPos/ncontacts)  
                       
        #NEW redo position update from scratch

        return task.cont
    
scene=SimpleScene()
scene.run()