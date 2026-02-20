from direct.showbase.ShowBase import ShowBase
from direct.showbase.ShowBaseGlobal import globalClock
from direct.task import Task
from panda3d.core import AmbientLight, DirectionalLight, Vec3

from panda_collision import CollisionWorld, Particle


class Collision2MassConservationApp(ShowBase):
    def __init__(self):
        super().__init__()
        self.disableMouse()

        self.camera.setPos(0.0, -35.0, 8.0)
        self.camera.lookAt(0.0, 25.0, 0.0)

        ambient = AmbientLight("ambient")
        ambient.setColor((0.7, 0.7, 0.7, 1.0))
        self.render.setLight(self.render.attachNewNode(ambient))

        sun = DirectionalLight("sun")
        sun.setColor((0.6, 0.6, 0.6, 1.0))
        sun_np = self.render.attachNewNode(sun)
        sun_np.setHpr(-30.0, -45.0, 0.0)
        self.render.setLight(sun_np)

        self.p_left = Particle(
            loader=self.loader,
            parent=self.render,
            pos=Vec3(-5.0, 25.0, 0.4),
            vel=Vec3(2.0, 0.0, 0.0),
            inverseMass=1.0,
            radius=1.0,
            color=(0.1, 0.6, 1.0, 1.0),
            name="left",
        )
        self.p_right = Particle(
            loader=self.loader,
            parent=self.render,
            pos=Vec3(5.0, 25.0, -0.4),
            vel=Vec3(-1.0, 0.0, 0.0),
            inverseMass=1.0,
            radius=1.0,
            color=(1.0, 0.4, 0.2, 1.0),
            name="right",
        )

        self.world = CollisionWorld([self.p_left, self.p_right], restitution=1.0)
        self.fixed_dt = 1.0 / 240.0
        self.accumulator = 0.0
        self.max_steps_per_frame = 32
        self.reported_collision = False
        self.reported_final = False

        print("Before collision:")
        self.world.calculateConservation()

        self.taskMgr.add(self.update_simulation, "update_simulation")

    def update_simulation(self, task: Task):
        frame_dt = min(globalClock.getDt(), 0.1)
        if frame_dt <= 0.0:
            return Task.cont

        self.accumulator += frame_dt
        hits = 0
        steps = 0
        while self.accumulator >= self.fixed_dt and steps < self.max_steps_per_frame:
            hits += self.world.step(self.fixed_dt)
            self.accumulator -= self.fixed_dt
            steps += 1

        if hits > 0 and not self.reported_collision:
            self.reported_collision = True
            print("\nImmediately after first collision:")
            self.world.calculateConservation()

        if self.world.time >= 8.0 and not self.reported_final:
            self.reported_final = True
            print("\nAfter simulation:")
            self.world.calculateConservation()
            print(f"Final left velocity  = {self.p_left.vel}")
            print(f"Final right velocity = {self.p_right.vel}")

        return Task.cont


if __name__ == "__main__":
    Collision2MassConservationApp().run()
