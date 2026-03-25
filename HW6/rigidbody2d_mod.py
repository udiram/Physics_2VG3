from panda3d.core import NodePath, Vec3


class Particle(NodePath):
    def __init__(self, name: str = "particle"):
        super().__init__(name)
        self.vel = Vec3(0.0, 0.0, 0.0)
        self.vold = Vec3(0.0, 0.0, 0.0)
        self.rold = Vec3(0.0, 0.0, 0.0)
        self.inverseMass = 1.0
        self.radius = 1.0
        self.collisionRadius = 1.8 * self.radius
        self.doGravity = False


class RigidBody2D(Particle):
    def __init__(self, name: str = "rigid_body_2d"):
        super().__init__(name)
        self.theta = 0.0
        self.thetaold = 0.0
        self.omega = 0.0
        self.inverseMomentOfInertia = self.inverseMass / (2.0 / 3.0 * self.radius * self.radius)
        self.shape = "square"
