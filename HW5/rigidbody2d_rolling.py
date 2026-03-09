from direct.showbase.ShowBase import ShowBase
from panda3d.core import *
from math import pi
from rigidbody2d_mod import *
from rigidbody2d_collision_mod import *

class SimpleScene(ShowBase):
    def __init__(self):
        super().__init__(self)

        # Extra features in this file:
        # - Cylinder/circle contacts via collideShapes (cylinder vs polygon).
        # - Rolling scenario default (cylinder, I = MR^2) on 13° incline.
        # - Race mode with 4 objects (hollow, solid, sphere, sliding particle).
        # - Finish speed printout for each race run.
        # - Tangential impulse direction corrected to oppose slip.

        # Use Orthographic projection (2D)
        lens = OrthographicLens()
        lens.setFilmSize(8.,6.)
        lens.setNearFar(-100,100)
        self.cam.node().setLens(lens)

        self.g = 10.0
        self.eCoeffRestitution = 0.2
        self.muFrictionStatic = 0
        self.muFrictionKinetic = 0

        self.dt = 1/60.
        self.t = 0.
        self.iFrame = 0
        self.fixPos = True  # Fix positions (normal only for rolling)
        self.fixVel = False

        self.cube = loader.loadModel("./panda3d/Cube.egg")
        self.cube.setScale(0.6)

        # Tube model for cylinder (nonagon in Panda3D)
        self.tube = loader.loadModel("./panda3d/Tube.egg")
        self.tube.setScale(1.3/1.41421356237)
        self.tube.setHpr(90.,90.,0.)
        self.tube.setPos(0,0.1,0)

        markerInit(self,"./panda3d/sphere.egg.pz")  # collision diagnostics

        self.RigidBody2Ds = []
        self.raceMode = 0
        self.printedFinish = False
        self.nextScenario()
        self.accept("n", self.nextScenario)

        self.gameTask = taskMgr.add(self.updateRigidBody2Ds, "updateRigidBody2Ds")

    def nextScenario(self):
        self.dt = 1/60

        for p in self.RigidBody2Ds:
            p.removeNode()
        self.muFrictionStatic = 0.5
        self.muFrictionKinetic = 0.5
        self.eCoeffRestitution = 1.0  # race: avoid energy loss from impacts
        self.fixPos = True  # default for rolling contacts
        self.printedFinish = False

        # Initialize RigidBody2Ds
        self.RigidBody2Ds = []
        for i in range(2):
            p = RigidBody2D("p")
            p.doGravity = False
            p.inverseMass = 1.0
            p.setScale(p.radius)
            p.setColor(Vec3(1,i,0))
            p.reparentTo(render)
            self.RigidBody2Ds.append(p)

        # Scenario: rolling cylinder on incline (default)
        angle = 13*pi/180

        # moving body (defaults, model assigned below)
        self.RigidBody2Ds[0].radius = 1.0
        self.RigidBody2Ds[0].collisionRadius = 1.1*self.RigidBody2Ds[0].radius
        self.RigidBody2Ds[0].setScale(1.0)
        self.RigidBody2Ds[0].setPos(-2.,3,self.RigidBody2Ds[0].radius-0.4)
        self.RigidBody2Ds[0].vel = Vec3(0.0,0.0,0.0)
        self.RigidBody2Ds[0].doGravity = True
        self.RigidBody2Ds[0].omega = 0
        self.RigidBody2Ds[0].shape = "cylinder"
        self.RigidBody2Ds[0].inverseMomentOfInertia = \
            self.RigidBody2Ds[0].inverseMass/(self.RigidBody2Ds[0].radius**2)

        # incline plane
        self.RigidBody2Ds[1].radius = 4.0
        self.RigidBody2Ds[1].collisionRadius = 1.8*self.RigidBody2Ds[1].radius
        self.RigidBody2Ds[1].setScale(4.0)
        self.RigidBody2Ds[1].setPos(0.,3,-self.RigidBody2Ds[1].radius-1.)
        self.RigidBody2Ds[1].vel = Vec3(0,0,0)
        self.RigidBody2Ds[1].inverseMass = 0.0
        self.RigidBody2Ds[1].inverseMomentOfInertia = 0.0
        self.RigidBody2Ds[1].theta = angle
        self.cube.instanceTo(self.RigidBody2Ds[1])

        # Race mode options (press 'n' to cycle)
        # 0: hollow cylinder, 1: solid cylinder, 2: sphere, 3: sliding particle
        if (self.raceMode == 0):
            k = 1.0
            label = "hollow cylinder"
            self.RigidBody2Ds[0].shape = "cylinder"
            self.tube.instanceTo(self.RigidBody2Ds[0])
            self.muFrictionStatic = 2.0
            self.muFrictionKinetic = 2.0
        elif (self.raceMode == 1):
            k = 0.5
            label = "solid cylinder"
            self.RigidBody2Ds[0].shape = "cylinder"
            self.tube.instanceTo(self.RigidBody2Ds[0])
            self.muFrictionStatic = 2.0
            self.muFrictionKinetic = 2.0
        elif (self.raceMode == 2):
            k = 2./5.
            label = "sphere"
            self.RigidBody2Ds[0].shape = "cylinder"
            self.tube.instanceTo(self.RigidBody2Ds[0])
            self.muFrictionStatic = 2.0
            self.muFrictionKinetic = 2.0
        else:
            k = 1.0
            label = "sliding particle"
            self.RigidBody2Ds[0].shape = "square"
            self.RigidBody2Ds[0].theta = angle
            self.RigidBody2Ds[0].collisionRadius = 1.8*self.RigidBody2Ds[0].radius
            self.muFrictionStatic = 0.0
            self.muFrictionKinetic = 0.0
            self.RigidBody2Ds[0].inverseMomentOfInertia = 0.0
            self.cube.instanceTo(self.RigidBody2Ds[0])
            # allow slight penetration so the particle can slide freely
            self.fixPos = False

        if (self.raceMode != 3):
            self.RigidBody2Ds[0].inverseMomentOfInertia = \
                self.RigidBody2Ds[0].inverseMass/(k*self.RigidBody2Ds[0].radius**2)

        print('raceMode=%i (%s)' % (self.raceMode, label))
        self.raceMode = (self.raceMode+1)%4

    def updateRigidBody2Ds(self, task):
        markerClean(self)

        # use precise timesteps
        dt = self.dt
        self.t += self.dt
        self.iFrame += 1

        for p in self.RigidBody2Ds:
            a = Vec3(0.,0.,0.)  # simple a here
            if (p.doGravity): a = Vec3(0.,0.,-self.g)
            p.vold = p.vel
            p.vel = p.vel + a*dt
            p.rold = p.getPos()
            r = p.rold + p.vel*dt
            p.setPos(r)
            p.theta += p.omega*dt
            p.setHpr( 90., p.theta*180/3.141592653589793238462, 0. )

        # Print speed when the moving body reaches the bottom (z <= 0)
        if (not self.printedFinish):
            p0 = self.RigidBody2Ds[0]
            if (p0.getPos().z <= 0.0):
                print('finish speed |v| = %.6f' % (p0.vel.length()))
                self.printedFinish = True

        # check other RigidBody2Ds in pairs (exclude self)
        for i,pi in enumerate(self.RigidBody2Ds):
            for pj in self.RigidBody2Ds[i+1:]:
                # collision detection
                xi = pi.getPos()
                xj = pj.getPos()

                rij = xj-xi
                # check 1: inside 3d sphere that encloses bodies
                if (abs(rij.length()) < pi.collisionRadius + pj.collisionRadius):
                    # Find closest points of contact
                    contacts = collideShapes( self, pi, pj )
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
                        # check 3: must be approaching
                        if (uij.dot(normal) < 0.):
                            ncontacts += 1
                            # resolve collision
                            nij = normal
                            dxi = (xclose-xi)
                            dxj = (xclose-xj)
                            rni = dxi.cross(nij)
                            rnj = dxj.cross(nij)

                            iMFac = 1/(pi.inverseMass+pj.inverseMass + \
                            rni.dot(rni)*pi.inverseMomentOfInertia + rnj.dot(rnj)*pj.inverseMomentOfInertia)
                            dpn = nij*(1+self.eCoeffRestitution)*uij.dot(nij)*iMFac

                            # Friction impulse (parallel to surface)
                            fric_vec = uij - nij*uij.dot(nij)
                            fric_len = fric_vec.length()
                            fric_ratio = fric_len/(abs(uij.dot(nij))+1e-20)
                            if (fric_len > 1e-20):
                                tdir = fric_vec / fric_len
                                rti = dxi.cross(tdir)
                                rtj = dxj.cross(tdir)
                                iMFacT = 1/(pi.inverseMass+pj.inverseMass + \
                                            rti.dot(rti)*pi.inverseMomentOfInertia + rtj.dot(rtj)*pj.inverseMomentOfInertia)
                                # Tangential impulse should reduce relative tangential velocity
                                dp_t = fric_vec * iMFacT

                                fric_mu = self.muFrictionStatic
                                if (fric_ratio > self.muFrictionStatic):
                                    fric_mu = self.muFrictionKinetic
                                fric_max = fric_mu*abs(dpn.dot(nij))
                                if (dp_t.length() > fric_max):
                                    dp_fric = dp_t.normalized()*fric_max
                                else:
                                    dp_fric = dp_t
                            else:
                                dp_fric = Vec3(0.,0.,0.)

                            dp = dpn + dp_fric

                            # Add effect of this contact to accumulator variables
                            dpiVel += dp*pi.inverseMass
                            dpjVel -= dp*pj.inverseMass
                            dpiOmega += (dxi.cross(dp)).y*pi.inverseMomentOfInertia
                            dpjOmega -= (dxj.cross(dp)).y*pj.inverseMomentOfInertia

                            if (self.fixPos):
                                # Normal position fix only (allow rolling)
                                dpiPos += nij*(pi.rold-xi).dot(nij)
                                dpjPos += nij*(pj.rold-xj).dot(nij)

                    # add accumulated change to rigid bodies
                    # Divide total effect by number of contacts
                    if (ncontacts > 0):
                        pi.vel += dpiVel/ncontacts
                        pj.vel += dpjVel/ncontacts
                        pi.omega += dpiOmega/ncontacts
                        pj.omega += dpjOmega/ncontacts
                        if (self.fixPos):
                            pi.setPos(xi + dpiPos/ncontacts)
                            pj.setPos(xj + dpjPos/ncontacts)

        return task.cont

scene=SimpleScene()
scene.run()
