from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from panda3d.bullet import BulletBoxShape, BulletRigidBodyNode, BulletSphereShape
from panda3d.core import BitMask32, CardMaker, Point3, TransformState, Vec3

from .config import (
    HEAVY_FIRE,
    PLAYER_COLORS,
    PROJECTILE_LIFETIME,
    RAPID_FIRE,
    SPHERE_VISUAL_SCALE,
    TANK_HALF_HEIGHT,
    TANK_HALF_WIDTH,
    TANK_LOWER_HALF_HEIGHT,
    TANK_MAX_HP,
    TANK_MASS,
    TANK_TURRET_PITCH_MAX,
    TANK_TURRET_PITCH_MIN,
    WEAPONS,
    WeaponSpec,
)


@dataclass
class TankState:
    player_id: int
    hp: int
    facing_sign: int
    turret_pitch_deg: float
    body_np: Any
    visual_np: Any
    turret_np: Any
    barrel_np: Any
    grounded: bool
    airborne: bool = False
    support_frames: int = 0
    movement_lock_s: float = 0.0
    knockback_velocity_x: float = 0.0
    cooldowns: dict[str, float] = field(default_factory=dict)


@dataclass
class ProjectileState:
    owner_id: int
    weapon_name: str
    body_np: Any
    last_pos: Vec3
    previous_velocity: Vec3
    lifetime_s: float
    crater_armed: bool
    damaged_targets: set[int] = field(default_factory=set)
    touching_ground: bool = False
    ground_time_s: float = 0.0

    @property
    def weapon(self) -> WeaponSpec:
        return WEAPONS[self.weapon_name]


def clamp_turret_pitch(angle_deg: float) -> float:
    return max(TANK_TURRET_PITCH_MIN, min(TANK_TURRET_PITCH_MAX, angle_deg))


def tick_cooldowns(cooldowns: dict[str, float], dt: float) -> dict[str, float]:
    return {name: max(0.0, time_left - dt) for name, time_left in cooldowns.items()}


def cooldown_remaining(cooldowns: dict[str, float], name: str) -> float:
    return max(0.0, cooldowns.get(name, 0.0))


def spawn_tank(
    world,
    render,
    assets: dict[str, Any],
    player_id: int,
    spawn_x: float,
    spawn_z: float,
    slope_angle_deg: float,
) -> TankState:
    facing_sign = 1 if player_id == 1 else -1

    lower_shape = BulletBoxShape(Vec3(TANK_HALF_WIDTH, 0.45, 0.30))
    upper_shape = BulletBoxShape(Vec3(0.875, 0.375, 0.24))

    body_np = render.attachNewNode(BulletRigidBodyNode(f"tank-{player_id}"))
    body = body_np.node()
    body.setMass(TANK_MASS)
    body.addShape(lower_shape)
    body.addShape(upper_shape, TransformState.makePos(Point3(0.0, 0.0, 0.575)))
    body.setFriction(0.38)
    body.setRestitution(0.05)
    body.setLinearDamping(0.18)
    body.setAngularDamping(0.92)
    body.setDeactivationEnabled(False)
    body.setLinearFactor(Vec3(1.0, 0.0, 1.0))
    body.setAngularFactor(Vec3(0.0, 1.0, 0.0))
    body_np.setPos(spawn_x, 0.0, spawn_z + TANK_HALF_HEIGHT + 0.09)
    body_np.setR(0.0)
    body_np.setCollideMask(BitMask32.allOn())
    body_np.setTag("kind", "tank")
    body_np.setPythonTag("player_id", player_id)
    world.attachRigidBody(body)

    visual_root = body_np.attachNewNode(f"tank-visual-{player_id}")
    visual_root.setR(slope_angle_deg)
    colors = PLAYER_COLORS[player_id]

    _copy_model(assets["cube"], visual_root, (0.0, 0.0, 0.0), (TANK_HALF_WIDTH * 2.0, 0.46, 0.31), colors["body"])
    _copy_model(assets["cube"], visual_root, (0.0, 0.0, 0.39), (1.725, 0.425, 0.275), colors["accent"])
    _copy_model(assets["cube"], visual_root, (-0.825, 0.0, -0.10), (0.41, 0.50, 0.20), (0.12, 0.16, 0.12, 1.0))
    _copy_model(assets["cube"], visual_root, (0.825, 0.0, -0.10), (0.41, 0.50, 0.20), (0.12, 0.16, 0.12, 1.0))

    turret_base = _copy_model(
        assets["cube"],
        visual_root,
        (0.0, 0.0, 0.71),
        (1.10, 0.66, 0.38),
        (0.08, 0.10, 0.08, 1.0),
    )
    _copy_model(
        assets["cube"],
        turret_base,
        (0.0, 0.0, 0.0),
        (1.18, 0.72, 0.42),
        colors["body"],
    )

    facing_np = turret_base.attachNewNode(f"tank-{player_id}-facing")
    if facing_sign < 0:
        facing_np.setH(180.0)

    turret_np = facing_np.attachNewNode(f"tank-{player_id}-turret")
    _make_barrel_card(
        turret_np,
        start_x=0.58,
        length=4.54,
        thickness=0.40,
        color=(0.05, 0.05, 0.05, 1.0),
        y_offset=0.03,
    )
    barrel_np = _make_barrel_card(
        turret_np,
        start_x=0.74,
        length=4.18,
        thickness=0.28,
        color=colors["accent"],
        y_offset=0.0,
    )
    muzzle_np = turret_np.attachNewNode(f"tank-{player_id}-muzzle")
    muzzle_np.setPos(4.92, 0.0, 0.0)
    barrel_np.setPythonTag("muzzle_np", muzzle_np)

    tank = TankState(
        player_id=player_id,
        hp=TANK_MAX_HP,
        facing_sign=facing_sign,
        turret_pitch_deg=28.0,
        body_np=body_np,
        visual_np=visual_root,
        turret_np=turret_np,
        barrel_np=barrel_np,
        grounded=False,
        airborne=False,
        support_frames=0,
        movement_lock_s=0.0,
        knockback_velocity_x=0.0,
        cooldowns={RAPID_FIRE.name: 0.0, HEAVY_FIRE.name: 0.0},
    )
    update_turret_visual(tank)
    return tank


def update_turret_visual(tank: TankState) -> None:
    tank.turret_pitch_deg = clamp_turret_pitch(tank.turret_pitch_deg)
    tank.turret_np.setR(-tank.turret_pitch_deg)


def spawn_projectile(
    world,
    render,
    assets: dict[str, Any],
    tank: TankState,
    weapon: WeaponSpec,
) -> ProjectileState:
    muzzle_np = tank.barrel_np.getPythonTag("muzzle_np")
    start = muzzle_np.getPos(render)
    pivot = tank.turret_np.getPos(render)
    direction = start - pivot
    if direction.length_squared() <= 1e-9:
        direction = Vec3(tank.facing_sign, 0.0, 0.0)
    direction.normalize()

    body_np = render.attachNewNode(BulletRigidBodyNode(f"{weapon.name}-shell-p{tank.player_id}"))
    body = body_np.node()
    body.setMass(weapon.mass)
    body.addShape(BulletSphereShape(weapon.radius))
    body.setLinearFactor(Vec3(1.0, 0.0, 1.0))
    body.setAngularFactor(Vec3(0.0, 1.0, 0.0))
    body.setFriction(0.95)
    body.setRestitution(0.72)
    body.setLinearDamping(0.05)
    body.setAngularDamping(0.12)
    body.setDeactivationEnabled(False)
    body.setCcdMotionThreshold(1e-7)
    body.setCcdSweptSphereRadius(max(0.15, weapon.radius * 0.75))
    body_np.setCollideMask(BitMask32.allOn())
    body_np.setPos(start)
    body_np.setTag("kind", "projectile")
    body_np.setPythonTag("owner_id", tank.player_id)
    body_np.setPythonTag("weapon_name", weapon.name)
    world.attachRigidBody(body)

    velocity = direction * weapon.muzzle_speed + tank.body_np.node().getLinearVelocity() * 0.2
    body.setLinearVelocity(velocity)

    _copy_model(
        assets["sphere"],
        body_np,
        (0.0, 0.0, 0.0),
        (weapon.radius * SPHERE_VISUAL_SCALE,) * 3,
        PLAYER_COLORS[tank.player_id]["projectile"],
    )

    return ProjectileState(
        owner_id=tank.player_id,
        weapon_name=weapon.name,
        body_np=body_np,
        last_pos=Vec3(start),
        previous_velocity=Vec3(velocity),
        lifetime_s=PROJECTILE_LIFETIME,
        crater_armed=True,
        ground_time_s=0.0,
    )


def apply_elastic_contact_impulse(
    body_a,
    body_b,
    normal: Vec3,
    restitution: float,
) -> bool:
    if normal.length_squared() <= 1e-9:
        return False
    normal = Vec3(normal)
    normal.normalize()

    inv_mass_a = _inverse_mass(body_a)
    inv_mass_b = _inverse_mass(body_b)
    if inv_mass_a + inv_mass_b <= 0.0:
        return False

    velocity_a = body_a.getLinearVelocity()
    velocity_b = body_b.getLinearVelocity()
    relative_velocity = velocity_b - velocity_a
    normal_speed = relative_velocity.dot(normal)
    if normal_speed >= 0.0:
        return False

    impulse_magnitude = -(1.0 + restitution) * normal_speed / (inv_mass_a + inv_mass_b)
    impulse = normal * impulse_magnitude
    if inv_mass_a > 0.0:
        body_a.applyCentralImpulse(-impulse)
        body_a.setActive(True)
    if inv_mass_b > 0.0:
        body_b.applyCentralImpulse(impulse)
        body_b.setActive(True)
    return True


def remove_projectile(world, projectile: ProjectileState) -> None:
    world.removeRigidBody(projectile.body_np.node())
    projectile.body_np.removeNode()


def _inverse_mass(body) -> float:
    mass = body.getMass()
    return 0.0 if mass <= 0.0 else 1.0 / mass


def _copy_model(model, parent, pos, scale, color):
    node = model.copyTo(parent)
    node.setPos(*pos)
    if isinstance(scale, tuple):
        node.setScale(*scale)
    else:
        node.setScale(scale)
    node.setColor(*color)
    return node


def _make_barrel_card(parent, start_x: float, length: float, thickness: float, color, y_offset: float):
    maker = CardMaker("tank-barrel")
    maker.setFrame(0.0, length, -thickness * 0.5, thickness * 0.5)
    node = parent.attachNewNode(maker.generate())
    node.setPos(start_x, y_offset, 0.0)
    node.setColor(*color)
    node.setTwoSided(True)
    return node
