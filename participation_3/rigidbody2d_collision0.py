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
        super().__init__()

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
            # Keep both bodies on the same z so the first hit is head-on (no angled deflection).
            p.setPos(Vec3(-6+17.*i,25,0.0))
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
                rij = xj - xi

                # check 1: possible overlap (simple spherical collision volume)
                dist = rij.length()
                minDist = pi.collisionRadius + pj.collisionRadius
                if dist == 0:
                    # avoid division by zero – choose arbitrary normal
                    n = Vec3(1, 0, 0)
                else:
                    n = rij / dist

                if dist < minDist:
                    # contact point (not used further here but kept for clarity)
                    contact_point = xi + n * pi.collisionRadius

                    # check 3: must be approaching along the contact normal
                    # n points from i -> j, so use (vj - vi) dot n
                    rel_vel = pj.vel - pi.vel
                    vn = rel_vel.dot(n)
                    if vn < 0.0:
                        # resolve collision with impulse-based response
                        invMass_i = pi.inverseMass
                        invMass_j = pj.inverseMass
                        j_mag = -(1.0 + self.eCoeffRestitution) * vn / (invMass_i + invMass_j)
                        # Vec3 * scalar is supported, scalar * Vec3 is not
                        impulse = n * j_mag

                        pi.vel = pi.vel - impulse * invMass_i
                        pj.vel = pj.vel + impulse * invMass_j

                        # positional correction to eliminate overlap
                        penetration = minDist - dist
                        if penetration > 0.0:
                            correction = n * (penetration / (invMass_i + invMass_j))
                            pi.setPos(xi - correction * invMass_i)
                            pj.setPos(xj + correction * invMass_j)

        return task.cont
    
scene=SimpleScene()
scene.run()
