from direct.showbase.ShowBase import ShowBase
from panda3d.core import DirectionalLight, AmbientLight, Vec4
from direct.interval.IntervalGlobal import Parallel
from direct.interval.LerpInterval import LerpHprInterval
import math

class BicycleScene(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        self.setup_bicycle()
        self.add_colors()
        self.setup_lighting()
        self.setup_ground()
        self.setup_circular_motion()
        self.setup_wheel_rotation()
        self.setup_camera()
        self.start_animation()

    def setup_bicycle(self):  # create the bicycle node with frame and wheels
        self.circle_centre = self.render.attachNewNode("circle_centre")  # node for circular motion
        self.bicycle = self.circle_centre.attachNewNode("bicycle")  # bicycle node

        self.frame = self.loader.loadModel("./panda3d/frame.egg")  # load frame model
        self.frame.reparentTo(self.bicycle)  # attach to bicycle node

        self.rear_wheel = self.loader.loadModel("./panda3d/wheel.egg")  # load rear wheel model
        self.rear_wheel.reparentTo(self.bicycle)  # attach to bicycle node
        self.rear_wheel.setPos(0, 0, 0)  # position at origin

        self.front_wheel = self.loader.loadModel("./panda3d/wheel.egg")  # load front wheel model
        self.front_wheel.reparentTo(self.bicycle)  # attach to bicycle node
        self.front_wheel.setPos(130, 0, 0)  # position 130 units forward

    def add_colors(self):  # add colors to make bicycle less washed-out
        self.frame.setColorScale(1.0, 0.2, 0.2, 1.0)  # strong red tint
        self.rear_wheel.setColorScale(0.1, 0.1, 0.1, 1.0)  # dark gray tint
        self.front_wheel.setColorScale(0.1, 0.1, 0.1, 1.0)

    def setup_lighting(self):  # add lighting to the scene
        ambient = AmbientLight("ambient")  # ambient light
        ambient.setColor(Vec4(0.4, 0.4, 0.4, 1))  # moderate intensity
        self.render.setLight(self.render.attachNewNode(ambient))  # attach to render

        sun = DirectionalLight("sun")  # directional light
        sun.setColor(Vec4(1.0, 1.0, 1.0, 1))  # bright white light
        sun_np = self.render.attachNewNode(sun)  # attach to render
        sun_np.setHpr(-30, -30, 0)  # angle to simulate sunlight
        self.render.setLight(sun_np)  # apply directional light

        self.render.setShaderAuto()  # Ensure lighting pipeline is enabled

    def setup_ground(self):  # create ground plane
        from panda3d.core import CardMaker
        cm = CardMaker("ground")  # create card maker
        cm.setFrame(-600, 600, -600, 600)  # large ground plane
        ground = self.render.attachNewNode(cm.generate())  # attach to render
        ground.setP(-90)  # rotate to be horizontal
        ground.setZ(-40)  # position below bicycle
        ground.setColor(0.25, 0.55, 0.25, 1.0)  # green color

    def setup_circular_motion(self):  # set up circular motion parameters
        self.circle_radius = 230   # large circle
        self.orbit_duration = 12   # orbit

        self.bicycle.setPos(self.circle_radius, 0, 0)  # position bicycle on circle
        self.bicycle.setH(90)  # face tangent to circle

    def setup_wheel_rotation(self):  # calculate wheel rotation intervals
        wheel_radius = 28  # estimated radius of the wheels

        orbit_distance = 2 * math.pi * self.circle_radius  # circumference of the orbit
        wheel_circumference = 2 * math.pi * wheel_radius  # circumference of the wheel
        num_rotations = orbit_distance / wheel_circumference  # number of wheel rotations
        total_rotation = num_rotations * 360  # total rotation in degrees

        self.rear_wheel_interval = LerpHprInterval(  # rear wheel rotation interval
            self.rear_wheel,  # node
            self.orbit_duration,  # duration
            (0, 0, total_rotation),  # end hpr
            (0, 0, 0)  # start hpr
        )

        self.front_wheel_interval = LerpHprInterval(  # front wheel rotation interval
            self.front_wheel,  # node
            self.orbit_duration,  # duration
            (0, 0, total_rotation),  # end hpr
            (0, 0, 0)  # start hpr
        )

        self.orbit_interval = LerpHprInterval(  # circular orbit interval
            self.circle_centre,  # node
            self.orbit_duration,  # duration
            (360, 0, 0),  # end hpr
            (0, 0, 0)  # start hpr
        )

    def setup_camera(self):  # position camera to view the scene
        self.camera.setPos(-100, -450, 180)  # position behind and above bicycle
        self.camera.lookAt(self.circle_centre)  # look at circle centre

    def start_animation(self):  # start all intervals in parallel
        Parallel(  # run rear wheel, front wheel, and orbit intervals together
            self.rear_wheel_interval,  # rear wheel rotation
            self.front_wheel_interval,  # front wheel rotation
            self.orbit_interval  # circular orbit
        ).loop()

if __name__ == "__main__":
    app = BicycleScene()
    app.run()