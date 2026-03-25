import sys
from direct.showbase.ShowBase import ShowBase
from direct.showbase.ShowBaseGlobal import globalClock
from direct.task import Task
from panda3d.core import AmbientLight, DirectionalLight, Vec3

from panda_collision import CollisionWorld, Particle

# configs: (p1_x, p2_x, wall_x, vx) - same vx for both, no overlap
CONFIGS = {
    "baseline": (-11.0, 0.0, 120.0, 10.0),
    "spread_out": (-50.0, -20.0, 120.0, 10.0),
    "closer_together": (-25.0, -12.0, 120.0, 10.0),
    "faster": (-11.0, 0.0, 120.0, 20.0),
    "slower": (-11.0, 0.0, 120.0, 5.0),
}


class Collision3MassesApp(ShowBase):
    def __init__(self, config_name="baseline"):
        super().__init__()
        self.disableMouse()
        self.config_name = config_name

        p1_x, p2_x, wall_x, vx = CONFIGS.get(config_name, CONFIGS["baseline"])

        if self.camera is not None:
            self.camera.setPos(-10.0, 120.0, 35.0)
            self.camera.lookAt(30.0, 200.0, 0.0)

        ambient = AmbientLight("ambient")
        ambient.setColor((0.75, 0.75, 0.75, 1.0))
        self.render.setLight(self.render.attachNewNode(ambient))

        sun = DirectionalLight("sun")
        sun.setColor((0.6, 0.6, 0.6, 1.0))
        sun_np = self.render.attachNewNode(sun)
        sun_np.setHpr(-15.0, -35.0, 0.0)
        self.render.setLight(sun_np)

        self.p1 = Particle(
            loader=self.loader,
            parent=self.render,
            pos=Vec3(p1_x, 200.0, 0.0),
            vel=Vec3(vx, 0.0, 0.0),
            inverseMass=1.0,
            radius=1.0,
            color=(0.95, 0.95, 1.0, 1.0),
            name="ping-pong",
        )
        self.p2 = Particle(
            loader=self.loader,
            parent=self.render,
            pos=Vec3(p2_x, 200.0, 0.0),
            vel=Vec3(vx, 0.0, 0.0),
            inverseMass=0.01,
            radius=10.0,
            color=(1.0, 0.5, 0.15, 0.8),
            name="basketball",
        )
        self.wall = Particle(
            loader=self.loader,
            parent=self.render,
            pos=Vec3(wall_x, 200.0, 0.0),
            vel=Vec3(0.0, 0.0, 0.0),
            inverseMass=0.0,
            radius=100.0,
            color=(0.3, 0.9, 0.35, 0.15),
            name="wall",
        )

        self.world = CollisionWorld([self.p1, self.p2, self.wall], restitution=1.0)
        self.fixed_dt = 1.0 / 240.0
        self.accumulator = 0.0
        self.max_steps_per_frame = 32
        self.reported = False

        self.taskMgr.add(self.update_simulation, "update_simulation")

    def update_simulation(self, task):
        frame_dt = min(globalClock.getDt(), 0.1)
        if frame_dt <= 0.0:
            return Task.cont

        self.accumulator += frame_dt
        steps = 0
        while self.accumulator >= self.fixed_dt and steps < self.max_steps_per_frame:
            self.world.step(self.fixed_dt)
            self.accumulator -= self.fixed_dt
            steps += 1

        if self.world.time >= 12.0 and not self.reported:
            self.reported = True
            p1_x, p2_x, wall_x, vx = CONFIGS.get(self.config_name, CONFIGS["baseline"])
            print(f"\n=== Config: {self.config_name} (p1_x={p1_x}, p2_x={p2_x}, wall_x={wall_x}, vx={vx}) ===")
            print(f"Final v1 (ping-pong)  = {self.p1.vel}")
            print(f"Final v2 (basketball) = {self.p2.vel}")
            print(f"Final v3 (wall)       = {self.wall.vel}")
            print(f"v1/vx = {self.p1.vel.x / vx:.4f},  v2/vx = {self.p2.vel.x / vx:.4f}")
            self.userExit()

        return Task.cont


if __name__ == "__main__":
    config = sys.argv[1] if len(sys.argv) > 1 else "baseline"
    Collision3MassesApp(config).run()
