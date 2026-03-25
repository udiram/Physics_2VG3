import csv
from pathlib import Path

from direct.showbase.ShowBase import ShowBase
from direct.showbase.ShowBaseGlobal import globalClock
from direct.task import Task
from panda3d.core import AmbientLight, DirectionalLight, Vec3

from panda_collision import CollisionWorld, Particle


class Collision2MassApp(ShowBase):
    def __init__(self):
        super().__init__()
        self.disableMouse()

        if self.camera is not None:
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
        self.csv_saved = False
        self._initial_row = None
        self._conservation_printed_after = False

        print("=== Before collision ===")
        self.world.calculateConservation()

        self.taskMgr.add(self.update_simulation, "update_simulation")

    def _gather_row_data(self, stage):
        # get pos, vel, P, K, L for csv
        pl, pr = self.p_left, self.p_right
        rl, rr = pl.getPos(), pr.getPos()
        vl, vr = pl.vel, pr.vel
        ml, mr = pl.mass, pr.mass
        P = vl * ml + vr * mr
        L = rl.cross(vl * ml) + rr.cross(vr * mr)
        K = 0.5 * ml * vl.dot(vl) + 0.5 * mr * vr.dot(vr)
        return {
            "stage": stage,
            "left_x": rl.x, "left_y": rl.y, "left_z": rl.z,
            "right_x": rr.x, "right_y": rr.y, "right_z": rr.z,
            "left_vx": vl.x, "left_vy": vl.y, "left_vz": vl.z,
            "right_vx": vr.x, "right_vy": vr.y, "right_vz": vr.z,
            "Px": P.x, "Py": P.y, "Pz": P.z,
            "K": K,
            "Lx": L.x, "Ly": L.y, "Lz": L.z,
        }

    def _save_csv(self, initial, final):
        csv_path = Path(__file__).parent / "collision_2.csv"
        headers = list(initial.keys())
        with open(csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=headers)
            w.writeheader()
            w.writerow(initial)
            w.writerow(final)
        self.csv_saved = True

    def update_simulation(self, task):
        frame_dt = min(globalClock.getDt(), 0.1)
        if frame_dt <= 0.0:
            return Task.cont

        # grab initial state before we step
        if self._initial_row is None:
            self._initial_row = self._gather_row_data("initial")

        self.accumulator += frame_dt
        steps = 0
        hits_this_frame = 0
        while self.accumulator >= self.fixed_dt and steps < self.max_steps_per_frame:
            hits_this_frame += self.world.step(self.fixed_dt)
            self.accumulator -= self.fixed_dt
            steps += 1

        # once we get a collision, call calculateConservation after and save csv
        if hits_this_frame > 0 and not self._conservation_printed_after:
            print("=== Immediately after first collision ===")
            self.world.calculateConservation()
            self._conservation_printed_after = True

        if hits_this_frame > 0 and not self.csv_saved and self._initial_row is not None:
            final_row = self._gather_row_data("final")
            self._save_csv(self._initial_row, final_row)

        return Task.cont


if __name__ == "__main__":
    Collision2MassApp().run()
