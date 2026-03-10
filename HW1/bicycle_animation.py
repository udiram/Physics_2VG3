from direct.showbase.ShowBase import ShowBase
from panda3d.core import DirectionalLight, AmbientLight, Vec4
from direct.interval.IntervalGlobal import Sequence, Parallel, Func
from direct.interval.LerpInterval import LerpHprInterval
import math

class BicycleScene(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        # Exercise 1: Create basic bicycle scene
        self.setup_bicycle()

        # Exercise 2: Add colors
        self.add_colors()

        # Exercise 5: Add lighting and ground
        self.setup_lighting()
        self.setup_ground()

        # Exercise 4: Setup circular motion
        self.setup_circular_motion()

        # Exercise 3 & 5: Setup wheel rotation with velocity matching
        self.setup_wheel_rotation()

        # Exercise 5: Setup camera
        self.setup_camera()

        # Start the animation
        self.start_animation()

    def setup_bicycle(self):
        """Exercise 1: Create the bicycle node with frame and wheels"""
        # Create circle centre node for circular motion
        self.circle_centre = self.render.attachNewNode("circle_centre")

        # Create bicycle node
        self.bicycle = self.circle_centre.attachNewNode("bicycle")

        # Load the frame model
        self.frame = self.loader.loadModel("./panda3d/frame.egg")
        self.frame.reparentTo(self.bicycle)

        # Load rear wheel (at origin)
        self.rear_wheel = self.loader.loadModel("./panda3d/wheel.egg")
        self.rear_wheel.reparentTo(self.bicycle)
        self.rear_wheel.setPos(0, 0, 0)

        # Load front wheel (130 units forward)
        self.front_wheel = self.loader.loadModel("./panda3d/wheel.egg")
        self.front_wheel.reparentTo(self.bicycle)
        self.front_wheel.setPos(130, 0, 0)

    def add_colors(self):
        """Exercise 2: Add colors to make bicycle less washed-out"""
        # Set blue color for the frame
        self.frame.setColor(0.2, 0.3, 1.0, 1.0)

        # Set darker color for wheels
        self.rear_wheel.setColor(0.3, 0.3, 0.3, 1.0)
        self.front_wheel.setColor(0.3, 0.3, 0.3, 1.0)

    def setup_lighting(self):
        """Exercise 5: Add lighting to show textures"""
        # Add ambient light
        ambient_light = AmbientLight("ambient_light")
        ambient_light.setColor(Vec4(0.4, 0.4, 0.4, 1))
        ambient_light_np = self.render.attachNewNode(ambient_light)
        self.render.setLight(ambient_light_np)

        # Add directional light (sun)
        directional_light = DirectionalLight("directional_light")
        directional_light.setColor(Vec4(0.8, 0.8, 0.8, 1))
        directional_light_np = self.render.attachNewNode(directional_light)
        directional_light_np.setHpr(45, -60, 0)
        self.render.setLight(directional_light_np)

    def setup_ground(self):
        """Exercise 5: Add ground plane"""
        # Create a simple ground plane
        from panda3d.core import CardMaker, TextureStage
        cm = CardMaker("ground")
        cm.setFrame(-500, 500, -500, 500)
        self.ground = self.render.attachNewNode(cm.generate())
        self.ground.setP(-90)  # Rotate to be horizontal
        self.ground.setZ(-30)  # Position below the bicycle
        self.ground.setColor(0.3, 0.6, 0.3, 1.0)  # Green ground

    def setup_circular_motion(self):
        """Exercise 4: Setup bicycle to move in a circle"""
        # Set bicycle offset from circle centre
        circle_radius = 200
        self.bicycle.setPos(circle_radius, 0, 0)

        # Orient bicycle to face forward in its circular path
        self.bicycle.setH(90)

        # Store radius for velocity calculations
        self.circle_radius = circle_radius
        self.orbit_duration = 10  # seconds for one complete orbit

    def setup_wheel_rotation(self):
        """Exercise 3 & 5: Setup wheel rotation with velocity matching"""
        # Calculate wheel rotation speed to match bicycle velocity
        # Bicycle linear velocity: v = 2πR / T where R is circle radius, T is period
        # Wheel must rotate such that its circumference distance matches bike distance
        # For a wheel of radius r, one rotation covers 2πr distance
        # Angular rotations needed: (2πR / T) / (2πr) rotations per second
        # In degrees per second: (2πR / T) / (2πr) * 360 = (R * 360) / (r * T)

        # Estimate wheel radius (approximately 30 units based on model)
        wheel_radius = 30

        # Calculate total distance bicycle travels in one orbit
        orbit_distance = 2 * math.pi * self.circle_radius

        # Calculate how many wheel rotations needed
        wheel_circumference = 2 * math.pi * wheel_radius
        num_rotations = orbit_distance / wheel_circumference

        # Total degrees to rotate
        total_wheel_rotation = num_rotations * 360

        # Create wheel rotation intervals
        # Wheels rotate around X-axis (roll) since they're positioned along X-axis
        self.rear_wheel_interval = LerpHprInterval(
            self.rear_wheel,
            self.orbit_duration,
            (0, 0, total_wheel_rotation),
            (0, 0, 0)
        )

        self.front_wheel_interval = LerpHprInterval(
            self.front_wheel,
            self.orbit_duration,
            (0, 0, total_wheel_rotation),
            (0, 0, 0)
        )

        # Create bicycle orbit interval
        self.orbit_interval = LerpHprInterval(
            self.circle_centre,
            self.orbit_duration,
            (360, 0, 0),
            (0, 0, 0)
        )

    def setup_camera(self):
        """Exercise 5: Position camera for good view"""
        # Position camera to follow the bicycle
        self.camera.setPos(0, -400, 150)
        self.camera.lookAt(self.circle_centre)

    def start_animation(self):
        """Start all animations in parallel"""
        # Run all intervals in parallel and loop them
        animation = Parallel(
            self.rear_wheel_interval,
            self.front_wheel_interval,
            self.orbit_interval
        )

        # Loop the animation forever
        animation.loop()

# Run the application
if __name__ == "__main__":
    app = BicycleScene()
    app.run()
