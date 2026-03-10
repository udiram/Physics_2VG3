from direct.showbase.ShowBase import ShowBase
from panda3d.core import *
import numpy as np

class Particle(NodePath):
    def __init__(self, *args, **kwargs):
        NodePath.__init__(self, *args, **kwargs)
        self.vel = Vec3(0.,0.,0.)
        self.inverseMass = 1.0
        self.force = Vec3(0.,0.,0.)

   
class SimpleScene(ShowBase):
    def __init__(self):
        super().__init__(self)

        self.printTimer = 0.0  # timer to track printing intervals

        dlight = DirectionalLight('dlight')
        dlnp = render.attachNewNode(dlight)           
        dlnp.setPos(50,0,100)
        dlnp.lookAt(0,50,0)
        render.setLight(dlnp)

        self.springLength = 15.
        self.springK = 10
        self.dampingC = 2.0 * np.sqrt(self.springK * 0.5)  # critical damping coefficient
        self.sphere = loader.loadModel("./panda3d/sphere.egg.pz")

        #Initialize particles
        self.particles = []
        for i in range(2):
            p = Particle("p")  
            self.sphere.instanceTo(p)
            p.setColor( 1.0 if i==0 else 0.0, 1.0 if i==1 else 0.0, 1.0 if i==2 else 0.0, 1.0)

            velocity = 46.5

            p.setPos(Vec3((i-0.5)*40.,100.,(i-0.5)*10.))  # (i-0.5*40) is -20 for i=0 and +20 for i=1, so they start 40 units apart
            p.vel = Vec3((i-0.5)*2*(velocity),0.,0.)
            p.reparentTo(render)
            self.particles.append(p)

        self.gameTask = taskMgr.add(self.updateParticles, "updateParticles")

    def updateParticles(self, task):
        dt = globalClock.getDt()
        dt = min(dt, 1 / 60.0)  # clamp big frame jumps
        self.printTimer += dt  # update timer

        for p in self.particles:  # reset forces
            p.force = Vec3(0, 0, 0)

        p0, p1 = self.particles[0], self.particles[1]  # spring between particles 0 and 1
        r01 = p1.getPos() - p0.getPos()
        dist = r01.length()

        if dist > 1e-6:
            n = r01 / dist

            vrel = p1.vel - p0.vel  # relative velocity along the spring
            v_parallel = vrel.dot(n)

            F0 = n * (self.springK * (dist - self.springLength) + self.dampingC * v_parallel)  # Get total force on particle 0

            p0.force += F0
            p1.force -= F0

        for p in self.particles:  # integrate (semi-implicit Euler)
            a = p.force * p.inverseMass
            p.vel = p.vel + a * dt
            p.setPos(p.getPos() + p.vel * dt)

        if self.printTimer >= 1.0:  # print every second
            separation = (self.particles[1].getPos() - self.particles[0].getPos()).length()  # calculate separation
            print(f"Separation: {separation:.3f}")
            self.printTimer -= 1.0  # keep remainder so timing stays accurate

        return task.cont
    
scene=SimpleScene()
scene.run()