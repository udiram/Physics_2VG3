from direct.showbase.ShowBase import ShowBase
from panda3d.core import *
from direct.gui.OnscreenText import OnscreenText
import math
from math import cos, sin
import copy
from rigidbody2d_mod import *
from rigidbody2d_markers_mod import *
from rigidbody2d_collide_mod import *
from rigidbody2d_constraints_mod import *

class SimpleScene(ShowBase):
    def __init__(self):
        super().__init__(self)

        #Use Orthographic projection (2D)
        lens = OrthographicLens()
        lens.setFilmSize(8.,6.)
        lens.setNearFar(-100,100)
        self.cam.node().setLens(lens)

        self.g = 10.0
        self.eCoeffRestitution = 1.0
        self.muFrictionStatic = 0.5
        self.muFrictionKinetic = 0.5

        self.dt = 1/60.
        self.t = 0.
        self.iFrame = 0
        self.iColl = 0
        self.iMarker = 0
        self.fixPos = True  #Fix positions
        self.fixVel = False
        self.doFriction = True

        self.cube = loader.loadModel("Cube")
        self.cube.setScale(0.6)  
        self.tube = loader.loadModel("Tube")
        self.tube.setScale(1.3/1.41421356237)  
        self.tube.setHpr(90.,90.,0.)
        self.tube.setPos(0,0.1,0)
        self.ball = loader.loadModel("samples/solar-system/models/planet_sphere")
        self.sphere = loader.loadModel("samples/solar-system/models/planet_sphere")
        self.sphere.setDepthTest(False); 

        #diagnostic
        markerInit(self,"samples/solar-system/models/planet_sphere")  #collision diagnostics 
 
        self.RigidBody2Ds = []
        self.constraintList = []  

        self.scenario = 0
        self.nextScenario()
  
        self.gameTask = taskMgr.add(self.updateRigidBody2Ds, "updateRigidBody2Ds")

    def bodyByName(scene, name):
        return [p for p in scene.RigidBody2Ds if p.name == name][0]
        
    def nextScenario(self):
        self.dt = 1/60

        for p in self.RigidBody2Ds: 
            p.removeNode() 

        #scenarios
        print('scenario=%i e=%f' % (self.scenario,self.eCoeffRestitution))
        
        #Initialize RigidBody2Ds
        self.RigidBody2Ds = []
        self.constraintList = []  
        angle = 0

        if (self.scenario == 0): #two blocks with hinge on plane
            p = RigidBody2D("plane")
            p.setColor(Vec3(1,0,0))
            p.radius = 2.0
            p.setScale(2.0)
            p.setPos(0.2,3,-p.radius-1.)
            p.vel = Vec3(0,0,0)
            p.inverseMass = 0.0
            p.inverseMomentOfInertia = 0.0
            p.reparentTo(render)
            self.cube.instanceTo(p) 
            self.RigidBody2Ds.append(p)

            p = RigidBody2D("block")
            p.name = "block1"
            p.radius = 0.5
            p.setScale(0.5)
            p.setPos(-0.,3,p.radius)
            p.vel = Vec3(-sin(angle),0,-cos(angle))
            p.doGravity = True
            p.inverseMomentOfInertia = p.inverseMass/(2./3.*p.radius**2)
            p.reparentTo(render)
            self.cube.instanceTo(p)
            self.RigidBody2Ds.append(p)
   
            p = RigidBody2D("block")
            p.name = "block2"
            p.radius = 0.3
            p.setScale(0.3)
            p.inverseMass = 1/0.5
            p.inverseMomentOfInertia = p.inverseMass/(2./3.*p.radius**2)
            p.setPos(-0.7,3,1.5)
            p.vel = Vec3(-sin(angle),0,-cos(angle))
            p.doGravity = True
            p.theta = -math.pi*0.4
            p.reparentTo(render)
            self.cube.instanceTo(p) 
            self.RigidBody2Ds.append(p)  

            pi = self.bodyByName( "block1" )
            pj = self.bodyByName( "block2")
            self.constraintList.append( ( "hinge", pi, pj, Vec3(-1,0,1)*pi.radius*1.01, Vec3(-1,0,-1)*pj.radius*1.01) )

        elif self.scenario==1:  #car
            self.eCoeffRestitution = 0.1
            angle = 0.10016742116 
            p = RigidBody2D("plane")
            p.radius = 4.0
            p.collisionRadius = 1.8*p.radius 
            p.setScale(4.0)
            p.setPos(0,3,-p.radius-1.5)
            p.theta = angle
            p.inverseMass = 0.0
            p.inverseMomentOfInertia = 0.0
            self.cube.instanceTo(p)
            p.reparentTo(render)
            self.RigidBody2Ds.append(p) 

            p = RigidBody2D("car")
            p.name = "car"
            p.radius = 1.0
            p.collisionRadius = 1.8*p.radius 
            p.setScale(1.0)
            p.setPos(-1.,3,1)
            p.doGravity = True
            p.setColor(Vec3(0,0.7,0.7))
            self.cube.instanceTo(p)
            p.reparentTo(render)
            self.RigidBody2Ds.append(p)
                
            p = RigidBody2D("wheel1")
            p.name = "wheel1"
            p.radius = 1.0
            p.collisionRadius = 1.1*p.radius 
            p.setScale(1.0)
            p.setPos(-2,3,0)
            p.doGravity = True
            p.shape = "cylinder"
            p.noCollideList = [ "car" ]
            p.inverseMomentOfInertia = p.inverseMass/(p.radius**2) 
            self.tube.instanceTo(p)    
            p.reparentTo(render)
            self.RigidBody2Ds.append(p)  
                        
            p = RigidBody2D("wheel2")
            p.name = "wheel2"
            p.radius = 0.8
            p.collisionRadius = 1.1*p.radius 
            p.setScale(0.8)
            p.setPos(0,3,0)
            p.doGravity = True
            p.shape = "cylinder" 
            p.noCollideList = [ "car" ]
            p.inverseMomentOfInertia = p.inverseMass/(p.radius**2)  
            self.tube.instanceTo(p)  
            p.reparentTo(render)
            self.RigidBody2Ds.append(p)          
 
            pi = self.bodyByName( "car" )
            pj = self.bodyByName( "wheel1")
            self.constraintList.append( ( "hinge", pi, pj, Vec3(-1,0,-1)*pi.radius, Vec3(0,0,0) ) )
            pj = self.bodyByName( "wheel2")
            self.constraintList.append( ( "hinge", pi, pj, Vec3(1,0,-1)*pi.radius, Vec3(0,0,0) ) )            
                
        #Make sure on screen rotation matches theta        
        for p in self.RigidBody2Ds:
            p.setHpr( 90., p.theta*180/math.pi, 0. )

    def updateRigidBody2Ds(self, task):
        markerClean( self )

        #use precise timesteps
        dt = self.dt
        self.t += self.dt
        self.iFrame += 1

        for p in self.RigidBody2Ds:
            a = Vec3(0.,0.,0.)  # simple a here
            if (p.doGravity): a = Vec3(0.,0.,-self.g)  
            p.vold = copy.deepcopy(p.vel)
            p.rold = p.getPos()
            p.thetaold = p.theta
            p.omegaold = p.omega

            p.vel = p.vel + a*dt  
            p.setPos( p.rold + p.vel*dt )
            p.theta += p.omega * dt
            p.setHpr( 90., p.theta*180/math.pi, 0. )


        constraint_contacts = constraintContactList( self )

        collision_contacts = collideContactList( self )
        
        niter = 100
        for iter in range(niter):
            for icon,con in enumerate(constraint_contacts):  
                # calculate impulse for constraint contacts
                dp, dummy, pi, pj, xci, xcj, xi, xj = con
                drij = xcj-xci
                dxi = (xci-xi)
                dxj = (xcj-xj)
                ui = pi.vel + Vec3(0.,pi.omega,0.).cross(dxi)
                uj = pj.vel + Vec3(0.,pj.omega,0.).cross(dxj)
                vbias = drij*.05/dt
                uij = (uj-ui) + vbias 
                nij = uij.normalized()
                rni = dxi.cross(nij)
                rnj = dxj.cross(nij)
                iMFac = 1/(pi.inverseMass+pj.inverseMass + \
                                rni.dot(rni)*pi.inverseMomentOfInertia + rnj.dot(rnj)*pj.inverseMomentOfInertia)      
                dpDelta = uij*iMFac
                dpNew = dp + dpDelta
                con[0] = dpNew

                pi.vel += (dpDelta)*pi.inverseMass
                pj.vel -= (dpDelta)*pj.inverseMass 
                
                pi.omega += (dxi.cross(dpDelta)).y*pi.inverseMomentOfInertia
                pj.omega -= (dxj.cross(dpDelta)).y*pj.inverseMomentOfInertia   
          
            for icontact,contact in enumerate(collision_contacts):                   
                # calculate impulse for collision contacts 
                dpScalar, dpFric, pi, pj, dclose, xclose, nij, uij0, vnormal, dxi, dxj, iMFac = contact
                ui = pi.vel + Vec3(0.,pi.omega,0.).cross(dxi)
                uj = pj.vel + Vec3(0.,pj.omega,0.).cross(dxj)
                uij = uj-ui
                unormal = uij.dot(nij)
                vbias = 0. if (dclose<0.01) else dclose*0.3/dt  #Erin Catto  
                delta_dpScalar = (unormal-(vnormal+vbias))*iMFac 
                dpScalarNew = dpScalar + delta_dpScalar
                if (dpScalarNew > 0.): dpScalarNew = 0.
                delta_dpScalar = dpScalarNew - dpScalar
                contact[0] = dpScalarNew
                if (self.doFriction): 
                    FricVecij = (uij)-nij*(uij).dot(nij)
                    FricDir = FricVecij.normalized()
                    rni = dxi.cross(FricDir)  #equivalent to dxi.cross(dp)/|dp|  friction will mess that up
                    rnj = dxj.cross(FricDir)
                    iMFacFric = 1/(pi.inverseMass+pj.inverseMass + \
                                rni.dot(rni)*pi.inverseMomentOfInertia + rnj.dot(rnj)*pj.inverseMomentOfInertia)
                    
                    delta_dpFric = FricVecij*iMFacFric   #effectively as above but target delta v // = 0
                    dpFricNew = dpFric + delta_dpFric
                    FricRatio = dpFricNew.length() / (abs(dpScalarNew)+1e-20)
                    if (FricRatio > self.muFrictionStatic):
                        dpFricNew = dpFricNew*self.muFrictionKinetic/FricRatio
                        FricRatio = self.muFrictionKinetic
                        delta_dpFric = dpFricNew - dpFric
                    contact[1] = dpFricNew

                pi.vel += (nij*delta_dpScalar+delta_dpFric)*pi.inverseMass
                pj.vel -= (nij*delta_dpScalar+delta_dpFric)*pj.inverseMass 
                
                pi.omega += (dxi.cross(nij*delta_dpScalar+delta_dpFric)).y*pi.inverseMomentOfInertia
                pj.omega -= (dxj.cross(nij*delta_dpScalar+delta_dpFric)).y*pj.inverseMomentOfInertia   

        if (len(collision_contacts)+len(constraint_contacts) > 0):
            self.iColl += 1
            #exit(1)

            if (self.fixPos):
                for p in self.RigidBody2Ds:   
                    p.setPos(p.rold + (p.vel+p.vold)*0.5*dt)
                    p.theta = p.thetaold + (p.omega+p.omegaold)*0.5*dt                       
        
        return task.cont
    
scene=SimpleScene()
scene.run()
