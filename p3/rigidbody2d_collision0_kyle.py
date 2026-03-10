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

        self.cube = loader.loadModel("./panda3d/Cube")
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
                delta = xj - xi

                # check 1: possible overlap (simple spherical collision volume)
                sep = delta.length()
                sum_r = pi.collisionRadius + pj.collisionRadius
                if sep <= 0.0:
                    print("Possible contact")
                    # avoid division by zero - choose arbitrary normal
                    normal = Vec3(1, 0, 0)
                else:
                    normal = delta / sep

                # check 2: contact point (not used further here but kept for clarity)
                if sep < sum_r:
                    print("Contact found")
                    _ = xi + normal * pi.collisionRadius

                    # check 3: must be approaching along the contact normal
                    # normal points from i -> j, so use (vj - vi) dot normal
                    v_rel = pj.vel - pi.vel
                    v_n = v_rel.dot(normal)
                    if v_n < 0.0:
                        print("Collision detected!")
                        # resolve collision with impulse-based response
                        inv_m_i = pi.inverseMass
                        inv_m_j = pj.inverseMass
                        j_imp = -(1.0 + self.eCoeffRestitution) * v_n / (inv_m_i + inv_m_j)
                        # Vec3 * scalar is supported, scalar * Vec3 is not
                        imp_vec = normal * j_imp

                        pi.vel = pi.vel - imp_vec * inv_m_i
                        pj.vel = pj.vel + imp_vec * inv_m_j

                        # positional correction to eliminate overlap
                        overlap = sum_r - sep
                        if overlap > 0.0:
                            shift = normal * (overlap / (inv_m_i + inv_m_j))
                            pi.setPos(xi - shift * inv_m_i)
                            pj.setPos(xj + shift * inv_m_j)

        return task.cont
    
scene=SimpleScene()
scene.run()
