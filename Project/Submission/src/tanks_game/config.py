from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class WeaponSpec:
    name: str
    cooldown_s: float
    damage: int
    radius: float
    mass: float
    muzzle_speed: float
    crater_radius_scale: float
    crater_depth_scale: float


ROOT_DIR = Path(__file__).resolve().parents[2]
ASSET_DIR = ROOT_DIR / "assets" / "models" / "panda3d"

WINDOW_TITLE = "2VG3 Tanks Hotseat"
SKY_COLOR = (0.47, 0.75, 0.98, 1.0)

TERRAIN_WIDTH_SAMPLES = 257
TERRAIN_DEPTH_SAMPLES = 17
TERRAIN_WORLD_WIDTH = TERRAIN_WIDTH_SAMPLES - 1
TERRAIN_WORLD_DEPTH = TERRAIN_DEPTH_SAMPLES - 1
TERRAIN_MAX_HEIGHT = 30.0
TERRAIN_KILL_Z = -22.0
TERRAIN_SPAWN_FRACTIONS = (0.20, 0.80)
TERRAIN_SPAWN_PAD_RADIUS = 8

FIXED_DT = 1.0 / 120.0
MAX_FRAME_DT = 0.25

CAMERA_FILM_WIDTH = 290.0
CAMERA_FILM_HEIGHT = 110.0
CAMERA_POS = (0.0, -180.0, 14.0)
CAMERA_LOOK_AT = (0.0, 0.0, 10.0)

WORLD_GRAVITY_Z = -28.0

TANK_MAX_HP = 100
TANK_HALF_WIDTH = 1.4
TANK_HALF_HEIGHT = 0.5
TANK_LOWER_HALF_HEIGHT = 0.30
TANK_MASS = 10.0
TANK_GROUND_ACCEL = 42.0
TANK_AIR_ACCEL = 14.0
TANK_MOVE_SPEED = 12.0
TANK_MAX_SPEED = 22.0
TANK_JUMP_IMPULSE = 110.0
TANK_TURRET_PITCH_MIN = -10.0
TANK_TURRET_PITCH_MAX = 190.0
TANK_TURRET_AIM_SPEED = 40.0
TANK_GROUNDED_EPSILON = 0.55
TANK_GROUND_SPEED_EPSILON = 4.0
TANK_CONTACT_RESTITUTION = 1.0
TANK_COLLISION_PUSH = 8.0
TANK_IDLE_DECEL = 6.0
TANK_COLLISION_BUFFER = 0.42
TANK_RIDE_HEIGHT = 0.12
TANK_SURFACE_SMOOTHING = 18.0
TANK_SUPPORT_PROBE = 0.55
TANK_KNOCKBACK_DECAY = 7.5

PROJECTILE_LIFETIME = 8.0
PROJECTILE_GROUND_DESPAWN_S = 2.2
PROJECTILE_TANK_RESTITUTION = 1.02
PROJECTILE_PROJECTILE_RESTITUTION = 1.04
PROJECTILE_KNOCKBACK_SCALE = 5.8
PROJECTILE_TANK_CONTROL_LOCK = 0.45
PROJECTILE_MIN_CRATER_IMPACT_SPEED = 10.0
PROJECTILE_REMOVAL_BUFFER = 28.0

SPHERE_VISUAL_SCALE = 1.9

RAPID_FIRE = WeaponSpec(
    name="rapid",
    cooldown_s=0.5,
    damage=5,
    radius=0.34,
    mass=1.0,
    muzzle_speed=70.0,
    crater_radius_scale=1.2,
    crater_depth_scale=0.55,
)

HEAVY_FIRE = WeaponSpec(
    name="heavy",
    cooldown_s=3.0,
    damage=25,
    radius=0.56,
    mass=2.8,
    muzzle_speed=58.0,
    crater_radius_scale=2.1,
    crater_depth_scale=1.2,
)

WEAPONS = {
    RAPID_FIRE.name: RAPID_FIRE,
    HEAVY_FIRE.name: HEAVY_FIRE,
}

PLAYER_CONTROLS = {
    1: {
        "left": "a",
        "right": "d",
        "aim_up": "w",
        "aim_down": "s",
        "rapid": "q",
        "heavy": "e",
        "jump": "lshift",
    },
    2: {
        "left": "j",
        "right": "l",
        "aim_up": "i",
        "aim_down": "k",
        "rapid": "u",
        "heavy": "o",
        "jump": "rshift",
    },
}

PLAYER_COLORS = {
    1: {
        "body": (0.25, 0.54, 0.20, 1.0),
        "accent": (0.77, 0.90, 0.42, 1.0),
        "projectile": (0.06, 0.06, 0.06, 1.0),
        "name": "Player 1",
    },
    2: {
        "body": (0.25, 0.36, 0.58, 1.0),
        "accent": (0.54, 0.82, 0.98, 1.0),
        "projectile": (0.06, 0.06, 0.06, 1.0),
        "name": "Player 2",
    },
}

PLAYER_NAMES = {player_id: data["name"] for player_id, data in PLAYER_COLORS.items()}
