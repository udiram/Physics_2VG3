from direct.showbase.ShowBase import ShowBase
from panda3d.core import *
from rigidbody2d_mod import *
from rigidbody2d_collision_mod import *

class SimpleScene(ShowBase):
    def __init__(self):
        super().__init__(self)

        # Extra features in this file (beyond rigidbody2d_friction.py):
        # - Simple sleep/resting state to suppress jitter at critical angles.
        # - Sleep triggers when linear/angular speeds stay below a threshold
        #   for several frames (cosmetic stability fix).

        #Use Orthographic projection (2D)
        lens = OrthographicLens()
        lens.setFilmSize(8.,6.)
        lens.setNearFar(-100,100)
        self.cam.node().setLens(lens)

        self.g = 10.0
        self.eCoeffRestitution = 0.1
        self.muFrictionStatic = 0.23
        self.muFrictionKinetic = 0.23

        self.dt = 1/60.
        self.t = 0.
        self.iFrame = 0
        self.fixPos = True  #Fix positions
        self.fixVel = False
        self.sleepFrames = 10
        self.sleepVelThresh = 3*self.g*self.dt

        self.cube = loader.loadModel("./panda3d/Cube.egg")
        self.cube.setScale(0.6)  

        markerInit(self,"./panda3d/sphere.egg.pz")  #collision diagnostics
 
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
            #inclined plane
            angle = 13*3.141592653589793238462/180
            self.RigidBody2Ds[0].radius = 0.5
            self.RigidBody2Ds[0].collisionRadius = 1.8*self.RigidBody2Ds[0].radius
            self.RigidBody2Ds[0].setScale(0.5)
            self.RigidBody2Ds[0].setPos(-0.,3,self.RigidBody2Ds[0].radius-0.4)
            self.RigidBody2Ds[0].vel = Vec3(0.0,0.0,0.0)
            self.RigidBody2Ds[0].doGravity = True
            self.RigidBody2Ds[0].inverseMomentOfInertia = \
                self.RigidBody2Ds[0].inverseMass/(2./3.*self.RigidBody2Ds[0].radius**2)

            self.RigidBody2Ds[1].radius = 2.0
            self.RigidBody2Ds[1].collisionRadius = 1.8*self.RigidBody2Ds[1].radius
            self.RigidBody2Ds[1].setScale(2.0)
            self.RigidBody2Ds[1].setPos(0.2,3,-self.RigidBody2Ds[1].radius-1.)
            self.RigidBody2Ds[1].vel = Vec3(0,0,0)
            self.RigidBody2Ds[1].inverseMass = 0.0
            self.RigidBody2Ds[1].inverseMomentOfInertia = 0.0

            self.RigidBody2Ds[0].theta = angle
            self.RigidBody2Ds[1].theta = angle
 
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
                    dpiVel = Vec3(0.,0.,0.)
                    dpjVel = Vec3(0.,0.,0.)
                    dpiOmega = 0.0
                    dpjOmega = 0.0
                    dpiPos = Vec3(0.,0.,0.)
                    dpjPos = Vec3(0.,0.,0.)
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
                            dpn = nij*(1+self.eCoeffRestitution)*uij.dot(nij)*iMFac

                            # Friction impulse (parallel to surface, opposes slip)
                            fric_vec = uij - nij*uij.dot(nij)
                            fric_len = fric_vec.length()
                            # fric_ratio approximates required friction: |u_t| / |u_n|
                            # Compare to mu_s to decide if we're inside the static friction cone.
                            fric_ratio = fric_len/(abs(uij.dot(nij))+1e-20)
                            fric_mu = self.muFrictionStatic
                            if (fric_ratio > self.muFrictionStatic):
                                fric_mu = self.muFrictionKinetic
                            if (fric_len > 1e-20):
                                fric_dir = fric_vec/ fric_len
                                fric_max = fric_mu*abs(dpn.dot(nij))
                                dp_fric = fric_dir*min(fric_max, fric_len*iMFac)
                            else:
                                dp_fric = Vec3(0.,0.,0.)

                            dp = dpn + dp_fric

                            # Add effect of this contact to accumulator variables
                            dpiVel += dp*pi.inverseMass
                            dpjVel -= dp*pj.inverseMass 
                            dpiOmega += (dxi.cross(dp)).y*pi.inverseMomentOfInertia
                            dpjOmega -= (dxj.cross(dp)).y*pj.inverseMomentOfInertia

                            if (self.fixPos):
                                # Normal position fix
                                dpiPos += nij*(pi.rold-xi).dot(nij)
                                dpjPos += nij*(pj.rold-xj).dot(nij)

                                # Parallel position fix (full if static, partial if sliding)
                                tfix_i = (pi.rold-xi) - nij*(pi.rold-xi).dot(nij)
                                tfix_j = (pj.rold-xj) - nij*(pj.rold-xj).dot(nij)
                                # Improvement (2c): full tangential correction only when inside static friction cone.
                                # Outside the cone (sliding), we scale the tangential correction so motion continues.
                                if (fric_ratio <= self.muFrictionStatic):
                                    dpiPos += tfix_i
                                    dpjPos += tfix_j
                                else:
                                    scale = min(1.0, self.muFrictionKinetic/(fric_ratio+1e-20))
                                    dpiPos += tfix_i*scale
                                    dpjPos += tfix_j*scale

                    #add accumulated change to rigid bodies
                    #Divide total effect by number of contacts
                    if (ncontacts > 0):
                        pi.vel += dpiVel/ncontacts
                        pj.vel += dpjVel/ncontacts
                        pi.omega += dpiOmega/ncontacts
                        pj.omega += dpjOmega/ncontacts
                        if (self.fixPos):
                            pi.setPos(xi + dpiPos/ncontacts)
                            pj.setPos(xj + dpjPos/ncontacts)

        # Improvement (2c): simple sleep/resting state to suppress jitter.
        # If linear + angular speeds stay below thresholds for several frames,
        # freeze the body by zeroing velocity and disabling gravity.
        for p in self.RigidBody2Ds:
            if (p.doGravity and p.vel.length() < self.sleepVelThresh and abs(p.omega) < self.sleepVelThresh):
                p.iFrameSleep += 1
                if (p.iFrameSleep >= self.sleepFrames):
                    p.vel = Vec3(0.,0.,0.)
                    p.omega = 0.0
                    p.doGravity = False
            else:
                p.iFrameSleep = 0
                        
        
        return task.cont
    
scene=SimpleScene()
scene.run()
