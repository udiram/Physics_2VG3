from direct.showbase.ShowBase import ShowBase
from panda3d.core import DirectionalLight, AmbientLight, Vec4, Vec3, CardMaker
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import TextNode
from direct.interval.IntervalGlobal import Parallel
from direct.interval.LerpInterval import LerpHprInterval
import math


class BicycleScene(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        # config
        self.wheel_r = 30
        self.path_r = 200
        self.direction = 1  # 1=CCW, -1=CW
        self.speed = 1.0
        self.cam_mode = "chase"

        self.anim = None
        self.cam_task = None
        self.smooth_pos = None
        self.smooth_look = None

        self._init_scene()
        self._init_controls()
        self._start()

    def _init_scene(self):
        # scene graph setup
        self.pivot = self.render.attachNewNode("pivot")
        self.bike = self.pivot.attachNewNode("bike")

        self.frame = self.loader.loadModel("frame.egg")
        self.frame.reparentTo(self.bike)
        self.frame.setColor(0.2, 0.3, 1.0, 1.0)

        self.rear = self.loader.loadModel("wheel.egg")
        self.rear.reparentTo(self.bike)
        self.rear.setColor(0.3, 0.3, 0.3, 1.0)

        self.front = self.loader.loadModel("wheel.egg")
        self.front.reparentTo(self.bike)
        self.front.setPos(130, 0, 0)
        self.front.setColor(0.3, 0.3, 0.3, 1.0)

        self.bike.setPos(self.path_r, 0, 0)
        self.bike.setH(90)

        # lighting
        amb = AmbientLight("amb")
        amb.setColor(Vec4(0.4, 0.4, 0.4, 1))
        self.render.setLight(self.render.attachNewNode(amb))

        sun = DirectionalLight("sun")
        sun.setColor(Vec4(0.8, 0.8, 0.8, 1))
        sun_np = self.render.attachNewNode(sun)
        sun_np.setHpr(45, -60, 0)
        self.render.setLight(sun_np)

        # ground plane
        cm = CardMaker("gnd")
        cm.setFrame(-500, 500, -500, 500)
        gnd = self.render.attachNewNode(cm.generate())
        gnd.setP(-90)
        gnd.setZ(-30)
        gnd.setColor(0.3, 0.6, 0.3, 1.0)

    def _init_controls(self):
        self.accept("1", self._cam_chase)
        self.accept("2", self._cam_overhead)
        self.accept("c", self._toggle_cam)
        self.accept("C", self._toggle_cam)
        self.accept("[", self._shrink_path)
        self.accept("]", self._grow_path)
        self.accept("arrow_left", self._shrink_path)
        self.accept("arrow_right", self._grow_path)
        for k in ["d", "D", "r", "R"]:
            self.accept(k, self._flip_dir)
        self.accept("-", self._slower)
        self.accept("=", self._faster)
        self.accept("arrow_down", self._slower)
        self.accept("arrow_up", self._faster)
        self.accept("escape", self.userExit)

        OnscreenText(
            "1/2 or C: camera  arrows or [ ]: radius  R/D: reverse  arrows or - =: speed",
            pos=(-1.3, -0.95), scale=0.05, fg=(1, 1, 1, 1), align=TextNode.ALeft
        )

    def _build_intervals(self, orbit_h=0, rear_r=0, front_r=0):
        dur = 10.0 / self.speed
        dist = 2 * math.pi * self.path_r
        wheel_spin = self.direction * (dist / (2 * math.pi * self.wheel_r)) * 360
        orbit_spin = self.direction * 360

        self.rear_iv = LerpHprInterval(
            self.rear, dur, (0, 0, rear_r + wheel_spin), (0, 0, rear_r)
        )
        self.front_iv = LerpHprInterval(
            self.front, dur, (0, 0, front_r + wheel_spin), (0, 0, front_r)
        )
        self.orbit_iv = LerpHprInterval(
            self.pivot, dur, (orbit_h + orbit_spin, 0, 0), (orbit_h, 0, 0)
        )

    def _start(self):
        self._build_intervals()
        self.anim = Parallel(self.rear_iv, self.front_iv, self.orbit_iv)
        self.anim.loop()
        self._apply_cam()

    def _restart(self):
        if self.anim:
            self.anim.pause()

        h = self.pivot.getH()
        rr = self.rear.getR()
        fr = self.front.getR()

        self.bike.setPos(self.path_r, 0, 0)
        self._build_intervals(h, rr, fr)

        self.anim = Parallel(self.rear_iv, self.front_iv, self.orbit_iv)
        self.anim.loop()

    # camera stuff

    def _apply_cam(self):
        if self.cam_task:
            self.taskMgr.remove(self.cam_task)
        if self.cam_mode == "chase":
            self.cam_task = self.taskMgr.add(self._chase_task, "cam")
        else:
            self.cam_task = self.taskMgr.add(self._overhead_task, "cam")

    def _chase_task(self, task):
        bike_pos = self.bike.getPos(self.render)
        center = self.pivot.getPos(self.render)

        radial = bike_pos - center
        radial.setZ(0)
        radial.normalize()

        fwd = Vec3(-radial.y, radial.x, 0) * self.direction

        tgt_pos = bike_pos - fwd * 250 + Vec3(0, 0, 100)
        tgt_look = bike_pos + fwd * 80 + Vec3(0, 0, 20)

        if self.smooth_pos is None:
            self.smooth_pos = Vec3(tgt_pos)
            self.smooth_look = Vec3(tgt_look)

        self.smooth_pos += (tgt_pos - self.smooth_pos) * 0.06
        self.smooth_look += (tgt_look - self.smooth_look) * 0.1

        self.camera.reparentTo(self.render)
        self.camera.setPos(self.smooth_pos)
        self.camera.lookAt(self.smooth_look)
        return task.cont

    def _overhead_task(self, task):
        c = self.pivot.getPos(self.render)
        h = max(700, self.path_r * 3 + 300)
        self.camera.reparentTo(self.render)
        self.camera.setPos(c.x, c.y, h)
        self.camera.lookAt(self.pivot)
        return task.cont

    def _cam_chase(self):
        self.cam_mode = "chase"
        self._apply_cam()

    def _cam_overhead(self):
        self.cam_mode = "overhead"
        self._apply_cam()

    def _toggle_cam(self):
        if self.cam_mode == "chase":
            self._cam_overhead()
        else:
            self._cam_chase()

    # controls

    def _grow_path(self):
        self.path_r = min(400, self.path_r + 25)
        self._restart()

    def _shrink_path(self):
        self.path_r = max(80, self.path_r - 25)
        self._restart()

    def _flip_dir(self):
        self.direction *= -1
        self.smooth_pos = None
        self.smooth_look = None
        self._restart()

    def _faster(self):
        self.speed = min(3.0, self.speed + 0.25)
        self._restart()

    def _slower(self):
        self.speed = max(0.25, self.speed - 0.25)
        self._restart()


if __name__ == "__main__":
    app = BicycleScene()
    app.run()
