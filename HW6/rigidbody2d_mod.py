from panda3d.core import *

class Particle(NodePath):
    def __init__(self, *args, **kwargs):
        NodePath.__init__(self, *args, **kwargs)
        self.vel = Vec3(0.,0.,0.)
        self.inverseMass = 1.0
        self.radius = 1.0 
        self.collisionRadius = 1.1
        self.doGravity = False
        self.iFrameSleep = 0

class RigidBody2D(Particle):
    def __init__(self, *args, **kwargs):
        Particle.__init__(self, *args, **kwargs)
        self.theta = 0.0 
        self.omega = 0.0
        self.inverseMomentOfInertia = self.inverseMass/(2./3.*self.radius**2)
        self.collisionRadius = 1.8*self.radius
        self.shape = "square"

