import random

from direct.showbase.ShowBase import ShowBase
from direct.showbase.ShowBaseGlobal import globalClock
from direct.task import Task
from panda3d.core import AmbientLight, DirectionalLight, Vec3

from panda_collision import CollisionWorld, Particle


class CollisionManyApp(ShowBase):
    def __init__(self):
        super().__init__()
        self.disableMouse()

        self.camera.setPos(0.0, 45.0, 20.0)
        self.camera.lookAt(0.0, 100.0, 0.0)

        ambient = AmbientLight("ambient")
        ambient.setColor((0.8, 0.8, 0.8, 1.0))
        self.render.setLight(self.render.attachNewNode(ambient))

        sun = DirectionalLight("sun")
        sun.setColor((0.55, 0.55, 0.55, 1.0))
        sun_np = self.render.attachNewNode(sun)
        sun_np.setHpr(-20.0, -50.0, 0.0)
        self.render.setLight(sun_np)

        self.min_corner = Vec3(-20.0, 80.0, -20.0)
        self.max_corner = Vec3(20.0, 120.0, 20.0)

        self.particles = self.make_random_particles(36, seed=2)
        self.world = CollisionWorld(self.particles, restitution=1.0, substeps=6, solver_iterations=4)
        self.fixed_dt = 1.0 / 240.0
        self.accumulator = 0.0
        self.max_steps_per_frame = 32
        self.reported = False
        self.print_clock = 0.0

        print("Exercise 3: start")
        self.world.calculateConservation()

        self.taskMgr.add(self.update_simulation, "update_simulation")

    def make_random_particles(self, n: int, seed: int) -> list[Particle]:
        random.seed(seed)
        particles: list[Particle] = []

        while len(particles) < n:
            pos = Vec3(
                random.uniform(-12.0, 12.0),
                random.uniform(88.0, 112.0),
                random.uniform(-12.0, 12.0),
            )
            radius = 1.25

            overlap = False
            for existing in particles:
                if (pos - existing.getPos()).length() < (radius + existing.radius):
                    overlap = True
                    break

            if overlap:
                continue

            particle = Particle(
                loader=self.loader,
                parent=self.render,
                pos=pos,
                vel=Vec3(
                    random.uniform(-10.0, 10.0),
                    random.uniform(-10.0, 10.0),
                    random.uniform(-10.0, 10.0),
                ),
                inverseMass=1.0,
                radius=radius,
                color=(
                    random.uniform(0.2, 1.0),
                    random.uniform(0.2, 1.0),
                    random.uniform(0.2, 1.0),
                    1.0,
                ),
                name=f"p{len(particles)}",
            )
            particles.append(particle)

        return particles

    def update_simulation(self, task: Task):
        frame_dt = min(globalClock.getDt(), 0.05)
        if frame_dt <= 0.0:
            return Task.cont

        self.accumulator += frame_dt
        steps = 0
        while self.accumulator >= self.fixed_dt and steps < self.max_steps_per_frame:
            self.world.step(self.fixed_dt)
            for particle in self.particles:
                self.world.bounce_in_box(particle, self.min_corner, self.max_corner)
            self.accumulator -= self.fixed_dt
            steps += 1

        if self.world.time - self.print_clock >= 5.0:
            self.print_clock = self.world.time
            print(f"t={self.world.time:.2f}s collisions={self.world.collision_count}")

        if self.world.time >= 30.0 and not self.reported:
            self.reported = True
            print("\nExercise 3: after 30 seconds")
            self.world.calculateConservation()
            print("30 second report complete; simulation continues.")

        return Task.cont


if __name__ == "__main__":
    CollisionManyApp().run()
