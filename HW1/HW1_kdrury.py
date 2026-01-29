from direct.showbase.ShowBase import ShowBase
from panda3d.core import DirectionalLight, AmbientLight, Vec4, CardMaker
from direct.interval.IntervalGlobal import Parallel
from direct.interval.LerpInterval import LerpHprInterval
import math

class HW1(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        self.build_bike()
        self.apply_colors()
        self.setup_lights()
        self.build_ground()
        self.setup_orbit_motion()
        self.setup_wheel_spin()
        self.setup_camera()
        self.start_animation()

    def build_bike(self):
        self.orbit_node = self.render.attachNewNode("orbit_node")  # Node that rotates to create circular motion

        self.bike_node = self.orbit_node.attachNewNode("bike_node")  # Bicycle root node

        self.frame_model = self.loader.loadModel("./panda3d/frame.egg")  # Load frame model
        self.frame_model.reparentTo(self.bike_node)  # Attach frame to bike node

        self.rear_wheel_model = self.loader.loadModel("./panda3d/wheel.egg")  # Load rear wheel model
        self.rear_wheel_model.reparentTo(self.bike_node)  # Attach rear wheel to bike node
        self.rear_wheel_model.setPos(0, 0, 0)  # Position rear wheel at origin

        self.front_wheel_model = self.loader.loadModel("./panda3d/wheel.egg")  # Load front wheel model
        self.front_wheel_model.reparentTo(self.bike_node)  # Attach front wheel to bike node
        self.front_wheel_model.setPos(130, 0, 0)  # Position front wheel 130 units forward

    def apply_colors(self):
        self.frame_model.setColorScale(1.0, 0.2, 0.2, 1.0)  # Red color for frame
        self.rear_wheel_model.setColorScale(0.1, 0.1, 0.1, 1.0)  # Dark color for rear wheel
        self.front_wheel_model.setColorScale(0.1, 0.1, 0.1, 1.0)  # Dark color for front wheel

    def setup_lights(self):
        ambient_light = AmbientLight("ambient_light")  # Create ambient light
        ambient_light.setColor(Vec4(0.4, 0.4, 0.4, 1))  # Set ambient light color
        ambient_np = self.render.attachNewNode(ambient_light)  # Attach to render
        self.render.setLight(ambient_np)  # Enable ambient light

        sun_light = DirectionalLight("sun_light")  # Create directional light
        sun_light.setColor(Vec4(1.0, 1.0, 1.0, 1))  # Set directional light color
        sun_np = self.render.attachNewNode(sun_light)  # Attach to render
        sun_np.setHpr(-30, -30, 0)  # Set direction of light
        self.render.setLight(sun_np)  # Enable directional light

        self.render.setShaderAuto()  # Enable lighting

    def build_ground(self):

        ground_maker = CardMaker("ground")  # Create ground card
        ground_maker.setFrame(-600, 600, -600, 600)  # Set ground size
        ground_np = self.render.attachNewNode(ground_maker.generate())  # Attach ground to render
        ground_np.setP(-90)  # Rotate to be horizontal
        ground_np.setZ(-40)  # Position below bicycle
        ground_np.setColor(0.25, 0.55, 0.25, 1.0)  # Green color for ground

    def setup_orbit_motion(self):
        self.orbit_radius = 230  # Radius of circular path
        self.orbit_time = 12  # Time to complete one orbit in seconds

        self.bike_node.setPos(self.orbit_radius, 0, 0)  # Position bike at orbit radius
        self.bike_node.setH(90)  # Face bike tangentially to path

    def setup_wheel_spin(self):
        wheel_radius_est = 28  # Estimated wheel radius

        path_length = 2 * math.pi * self.orbit_radius  # Circumference of circular path
        wheel_circ = 2 * math.pi * wheel_radius_est  # Wheel circumference
        rotations_needed = path_length / wheel_circ  # Number of wheel rotations needed
        total_degrees = rotations_needed * 360  # Total degrees to spin wheels

        self.rear_spin_interval = LerpHprInterval(
            self.rear_wheel_model,  # Rear wheel spin interval
            self.orbit_time,      # Duration of spin
            (0, 0, total_degrees),  # End HPR
            (0, 0, 0)  # Start HPR
        )

        self.front_spin_interval = LerpHprInterval(
            self.front_wheel_model,  # Front wheel spin interval
            self.orbit_time,    # Duration of spin
            (0, 0, total_degrees),  # End HPR
            (0, 0, 0)  # Start HPR
        )

        self.orbit_spin_interval = LerpHprInterval(
            self.orbit_node,  # Orbit node spin interval
            self.orbit_time,  # Duration of orbit
            (360, 0, 0),  # End HPR
            (0, 0, 0)  # Start HPR
        )

    def setup_camera(self):
        self.camera.setPos(-100, -450, 180)  # Position camera
        self.camera.lookAt(self.orbit_node)  # Look at orbit center

    def start_animation(self):
        Parallel(
            self.rear_spin_interval,  # Rear wheel spin
            self.front_spin_interval,  # Front wheel spin
            self.orbit_spin_interval  # Orbit motion
        ).loop()

if __name__ == "__main__":
    app = HW1()
    app.run()