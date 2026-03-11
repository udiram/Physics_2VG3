from __future__ import annotations

from math import cos, radians, sin
import random

from direct.showbase.ShowBase import ShowBase
from panda3d.bullet import BulletDebugNode, BulletWorld
from panda3d.core import (
    AmbientLight,
    DirectionalLight,
    Filename,
    OrthographicLens,
    Vec3,
    Vec4,
    loadPrcFileData,
)

from .combat import (
    ProjectileState,
    TankState,
    apply_elastic_contact_impulse,
    clamp_turret_pitch,
    cooldown_remaining,
    remove_projectile,
    spawn_projectile,
    spawn_tank,
    tick_cooldowns,
    update_turret_visual,
)
from .config import (
    ASSET_DIR,
    CAMERA_FILM_HEIGHT,
    CAMERA_FILM_WIDTH,
    CAMERA_LOOK_AT,
    CAMERA_POS,
    FIXED_DT,
    MAX_FRAME_DT,
    PLAYER_CONTROLS,
    PLAYER_NAMES,
    PROJECTILE_KNOCKBACK_SCALE,
    PROJECTILE_GROUND_DESPAWN_S,
    PROJECTILE_MIN_CRATER_IMPACT_SPEED,
    PROJECTILE_PROJECTILE_RESTITUTION,
    PROJECTILE_REMOVAL_BUFFER,
    PROJECTILE_TANK_CONTROL_LOCK,
    PROJECTILE_TANK_RESTITUTION,
    SKY_COLOR,
    TANK_AIR_ACCEL,
    TANK_COLLISION_BUFFER,
    TANK_COLLISION_PUSH,
    TANK_CONTACT_RESTITUTION,
    TANK_GROUND_ACCEL,
    TANK_GROUNDED_EPSILON,
    TANK_GROUND_SPEED_EPSILON,
    TANK_HALF_HEIGHT,
    TANK_HALF_WIDTH,
    TANK_IDLE_DECEL,
    TANK_JUMP_IMPULSE,
    TANK_KNOCKBACK_DECAY,
    TANK_LOWER_HALF_HEIGHT,
    TANK_MAX_SPEED,
    TANK_MOVE_SPEED,
    TANK_RIDE_HEIGHT,
    TANK_SUPPORT_PROBE,
    TANK_SURFACE_SMOOTHING,
    TANK_TURRET_AIM_SPEED,
    WEAPONS,
    WINDOW_TITLE,
    WORLD_GRAVITY_Z,
)
from .hud import GameHud
from .terrain import (
    TerrainState,
    carve_crater,
    create_terrain_state,
    pixel_to_world_x,
    rebuild_terrain_body,
    sample_surface_height,
    surface_angle_degrees,
    surface_normal_at_x,
)


loadPrcFileData("", f"window-title {WINDOW_TITLE}")


class TanksHotseatGame(ShowBase):
    def __init__(self):
        super().__init__()
        self.disableMouse()
        self.setBackgroundColor(*SKY_COLOR)
        self.setFrameRateMeter(True)

        self.assets = self._load_assets()
        self.hud = GameHud()
        self.debug_np = None
        self.world = None
        self.terrain: TerrainState | None = None
        self.tanks: list[TankState] = []
        self.projectiles: list[ProjectileState] = []
        self.active_contact_pairs: set[tuple[int, int]] = set()
        self.key_state: dict[str, bool] = {}
        self.button_states: dict[str, bool] = {}
        self.accumulator = 0.0
        self.rng = random.Random()
        self.match_over = False
        self.winner_text: str | None = None

        self._configure_camera()
        self._configure_lighting()
        self._bind_events()
        self.reset_match()
        self.taskMgr.add(self._update_task, "update-game")

    def reset_match(self):
        self._clear_world()
        self.projectiles = []
        self.tanks = []
        self.active_contact_pairs.clear()
        for key in list(self.key_state):
            self.key_state[key] = False
        self.button_states.clear()
        self.accumulator = 0.0
        self.match_over = False
        self.winner_text = None

        self.world = BulletWorld()
        self.world.setGravity(Vec3(0.0, 0.0, WORLD_GRAVITY_Z))

        debug_node = BulletDebugNode("debug")
        self.debug_np = self.render.attachNewNode(debug_node)
        self.debug_np.hide()
        self.world.setDebugNode(debug_node)

        self.terrain = create_terrain_state(self.world, self.render, self.camera, self.rng)

        for index, pixel in enumerate(self.terrain.spawn_pixels, start=1):
            spawn_x = pixel_to_world_x(pixel, self.terrain.image)
            spawn_z = sample_surface_height(self.terrain, spawn_x)
            slope_angle = surface_angle_degrees(self.terrain, spawn_x)
            tank = spawn_tank(
                self.world,
                self.render,
                self.assets,
                index,
                spawn_x,
                spawn_z,
                slope_angle,
            )
            self.tanks.append(tank)
            self._align_tank_to_surface(tank, spawn_x)

        self._refresh_hud()

    def destroy(self):
        self._clear_world()
        self.hud.destroy()
        super().destroy()

    def _load_assets(self):
        cube_path = Filename.fromOsSpecific(str(ASSET_DIR / "Cube.egg"))
        sphere_path = Filename.fromOsSpecific(str(ASSET_DIR / "sphere.egg.pz"))
        return {
            "cube": self.loader.loadModel(cube_path),
            "sphere": self.loader.loadModel(sphere_path),
        }

    def _configure_camera(self):
        lens = OrthographicLens()
        lens.setFilmSize(CAMERA_FILM_WIDTH, CAMERA_FILM_HEIGHT)
        lens.setNearFar(-250.0, 250.0)
        self.cam.node().setLens(lens)
        self.cam.setPos(*CAMERA_POS)
        self.cam.lookAt(*CAMERA_LOOK_AT)

    def _configure_lighting(self):
        ambient = AmbientLight("ambient")
        ambient.setColor(Vec4(0.60, 0.60, 0.62, 1.0))
        ambient_np = self.render.attachNewNode(ambient)

        directional = DirectionalLight("directional")
        directional.setDirection(Vec3(0.4, 0.2, -1.0))
        directional.setColor(Vec4(0.94, 0.94, 0.90, 1.0))
        directional_np = self.render.attachNewNode(directional)

        self.render.clearLight()
        self.render.setLight(ambient_np)
        self.render.setLight(directional_np)

    def _bind_events(self):
        self.accept("escape", self.userExit)
        self.accept("r", self.reset_match)
        self.accept("f3", self._toggle_debug)
        for player_controls in PLAYER_CONTROLS.values():
            for action, key in player_controls.items():
                self._bind_key(key)

    def _toggle_debug(self):
        if self.debug_np.isHidden():
            self.debug_np.show()
        else:
            self.debug_np.hide()

    def _update_task(self, task):
        frame_dt = min(globalClock.getDt(), MAX_FRAME_DT)
        self.accumulator += frame_dt
        while self.accumulator >= FIXED_DT:
            self._fixed_update(FIXED_DT)
            self.accumulator -= FIXED_DT
        return task.cont

    def _fixed_update(self, dt: float):
        if not self.world or not self.terrain:
            return

        if not self.match_over:
            self._process_player_inputs(dt)

        for projectile in self.projectiles:
            projectile.last_pos = Vec3(projectile.body_np.getPos())
            projectile.previous_velocity = Vec3(projectile.body_np.node().getLinearVelocity())

        self.world.doPhysics(dt, 1, dt)
        self._update_grounding(dt)
        self._clamp_tank_speeds()
        self._resolve_tank_tank_collision()
        self._update_projectiles(dt)
        if not self.match_over:
            self._handle_contacts()
            self._check_win_state()
        self._refresh_hud()

    def _process_player_inputs(self, dt: float):
        for tank in self.tanks:
            controls = PLAYER_CONTROLS[tank.player_id]
            tank.cooldowns = tick_cooldowns(tank.cooldowns, dt)
            tank.movement_lock_s = max(0.0, tank.movement_lock_s - dt)

            move_dir = 0
            if tank.movement_lock_s > 0.0 and abs(tank.knockback_velocity_x) > 0.01:
                self._apply_knockback_motion(tank, dt)
            elif tank.movement_lock_s <= 0.0:
                tank.knockback_velocity_x = 0.0
                if self._is_key_down(controls["left"]):
                    move_dir -= 1
                if self._is_key_down(controls["right"]):
                    move_dir += 1
                self._apply_horizontal_movement(tank, move_dir, dt)

            aim_delta = 0.0
            if self._is_key_down(controls["aim_up"]):
                aim_delta += TANK_TURRET_AIM_SPEED * dt
            if self._is_key_down(controls["aim_down"]):
                aim_delta -= TANK_TURRET_AIM_SPEED * dt
            if aim_delta:
                tank.turret_pitch_deg = clamp_turret_pitch(tank.turret_pitch_deg + aim_delta)
                update_turret_visual(tank)

            if self._pressed_once(controls["jump"]) and tank.grounded:
                tank.body_np.node().applyCentralImpulse(Vec3(0.0, 0.0, TANK_JUMP_IMPULSE))
                tank.body_np.node().setActive(True)
                tank.grounded = False
                tank.airborne = True
                tank.support_frames = 0

            if self._is_key_down(controls["rapid"]):
                self._try_fire_weapon(tank, WEAPONS["rapid"])
            if self._is_key_down(controls["heavy"]):
                self._try_fire_weapon(tank, WEAPONS["heavy"])

    def _try_fire_weapon(self, tank: TankState, weapon):
        if cooldown_remaining(tank.cooldowns, weapon.name) > 0.0:
            return
        projectile = spawn_projectile(self.world, self.render, self.assets, tank, weapon)
        self.projectiles.append(projectile)
        tank.cooldowns[weapon.name] = weapon.cooldown_s

    def _update_grounding(self, dt: float):
        for tank in self.tanks:
            surface_height = self._support_surface_height(tank.body_np.getX())
            body_bottom = tank.body_np.getZ() - TANK_HALF_HEIGHT
            vertical_speed = tank.body_np.node().getLinearVelocity().z
            supported_now = (
                surface_height is not None
                and self._has_support_at(tank.body_np.getX())
                and
                body_bottom <= surface_height + TANK_GROUNDED_EPSILON
                and vertical_speed <= TANK_GROUND_SPEED_EPSILON
            )
            if supported_now:
                tank.support_frames = min(6, tank.support_frames + 1)
            else:
                tank.support_frames = max(0, tank.support_frames - 1)

            tank.grounded = tank.support_frames >= 2
            if tank.grounded:
                target_z = surface_height + TANK_HALF_HEIGHT + TANK_RIDE_HEIGHT
                current_z = tank.body_np.getZ()
                smooth_factor = min(1.0, TANK_SURFACE_SMOOTHING * dt)
                tank.body_np.setZ(current_z + (target_z - current_z) * smooth_factor)
                velocity = tank.body_np.node().getLinearVelocity()
                if velocity.z < 0.0:
                    velocity.z *= 0.25
                    tank.body_np.node().setLinearVelocity(velocity)
                self._align_tank_to_surface(tank, tank.body_np.getX())
                tank.airborne = False
            elif surface_height is None or vertical_speed < -0.1:
                tank.airborne = True

    def _clamp_tank_speeds(self):
        for tank in self.tanks:
            velocity = tank.body_np.node().getLinearVelocity()
            velocity.y = 0.0
            velocity.x = max(-TANK_MAX_SPEED, min(TANK_MAX_SPEED, velocity.x))
            tank.body_np.node().setLinearVelocity(velocity)

    def _update_projectiles(self, dt: float):
        alive: list[ProjectileState] = []
        for projectile in self.projectiles:
            projectile.lifetime_s -= dt
            if projectile.touching_ground:
                projectile.ground_time_s += dt
            else:
                projectile.ground_time_s = 0.0
            if projectile.lifetime_s <= 0.0:
                remove_projectile(self.world, projectile)
                continue
            if projectile.ground_time_s >= PROJECTILE_GROUND_DESPAWN_S:
                remove_projectile(self.world, projectile)
                continue

            pos = projectile.body_np.getPos()
            if (
                pos.z < self.terrain.kill_z - PROJECTILE_REMOVAL_BUFFER
                or abs(pos.x) > self.terrain.world_width / 2.0 + PROJECTILE_REMOVAL_BUFFER
            ):
                remove_projectile(self.world, projectile)
                continue
            alive.append(projectile)
        self.projectiles = alive

    def _handle_contacts(self):
        current_pairs: set[tuple[int, int]] = set()
        handled_new_pairs: set[tuple[int, int]] = set()
        crater_requests: list[tuple[ProjectileState, float, float]] = []
        projectiles_to_remove: set[int] = set()

        projectile_nodes = {
            self._node_key(projectile.body_np.node()): projectile for projectile in self.projectiles
        }
        tank_nodes = {self._node_key(tank.body_np.node()): tank for tank in self.tanks}

        for projectile in self.projectiles:
            projectile_key = self._node_key(projectile.body_np.node())
            if projectile_key in projectiles_to_remove:
                continue
            terrain_touch = False
            result = self.world.contactTest(projectile.body_np.node())
            for contact in result.getContacts():
                node_a = contact.getNode0()
                node_b = contact.getNode1()
                other = (
                    node_b
                    if self._node_key(node_a) == self._node_key(projectile.body_np.node())
                    else node_a
                )
                pair = self._pair_key(projectile.body_np.node(), other)
                current_pairs.add(pair)

                if pair in self.active_contact_pairs or pair in handled_new_pairs:
                    if other.getName() == "terrain":
                        terrain_touch = True
                    continue

                other_key = self._node_key(other)
                if other_key in tank_nodes:
                    tank = tank_nodes[other_key]
                    self._handle_projectile_tank_contact(projectile, tank)
                    projectiles_to_remove.add(projectile_key)
                    handled_new_pairs.add(pair)
                    break
                elif other_key in projectile_nodes:
                    other_projectile = projectile_nodes[other_key]
                    if projectile is not other_projectile:
                        self._handle_projectile_projectile_contact(projectile, other_projectile)
                        handled_new_pairs.add(pair)
                elif other.getName() == "terrain":
                    terrain_touch = True
                    if projectile.crater_armed:
                        impact_speed = self._projectile_ground_impact_speed(projectile)
                        crater_requests.append((projectile, projectile.body_np.getX(), impact_speed))
                        handled_new_pairs.add(pair)
            if terrain_touch:
                projectile.touching_ground = True
                projectile.crater_armed = False
            else:
                projectile.touching_ground = False
                projectile.crater_armed = True

        for projectile, world_x, impact_speed in crater_requests:
            if impact_speed < PROJECTILE_MIN_CRATER_IMPACT_SPEED:
                continue
            if carve_crater(
                self.terrain,
                world_x,
                projectile.weapon.radius,
                impact_speed,
                projectile.weapon.crater_radius_scale,
                projectile.weapon.crater_depth_scale,
            ):
                rebuild_terrain_body(self.world, self.terrain)
                self._reactivate_nearby_bodies(world_x, projectile.weapon.radius * 8.0)

        if projectiles_to_remove:
            survivors: list[ProjectileState] = []
            for projectile in self.projectiles:
                if self._node_key(projectile.body_np.node()) in projectiles_to_remove:
                    remove_projectile(self.world, projectile)
                    continue
                survivors.append(projectile)
            self.projectiles = survivors

        self.active_contact_pairs = current_pairs

    def _handle_projectile_tank_contact(self, projectile: ProjectileState, tank: TankState):
        normal = tank.body_np.getPos() - projectile.body_np.getPos()
        normal.y = 0.0
        if normal.length_squared() <= 1e-9:
            normal = Vec3(1.0 if projectile.owner_id != tank.player_id else -1.0, 0.0, 0.0)
        apply_elastic_contact_impulse(
            projectile.body_np.node(),
            tank.body_np.node(),
            normal,
            PROJECTILE_TANK_RESTITUTION,
        )
        normal.normalize()
        impact_speed = abs(projectile.previous_velocity.dot(normal))
        knockback = (
            projectile.weapon.mass * impact_speed * 0.50
            + projectile.weapon.damage * PROJECTILE_KNOCKBACK_SCALE
        )
        tank.body_np.node().applyCentralImpulse(normal * knockback)
        tank_velocity = tank.body_np.node().getLinearVelocity()
        if abs(normal.x) > 1e-6:
            direction = 1.0 if normal.x >= 0.0 else -1.0
            minimum_speed = min(TANK_MAX_SPEED, projectile.weapon.damage * 1.1 + impact_speed * 0.18)
            if direction * tank_velocity.x < minimum_speed:
                tank_velocity.x = direction * minimum_speed
                tank.body_np.node().setLinearVelocity(tank_velocity)
            tank.knockback_velocity_x = direction * minimum_speed
        tank.body_np.node().setActive(True)
        tank.movement_lock_s = max(tank.movement_lock_s, PROJECTILE_TANK_CONTROL_LOCK)
        tank.hp = max(0, tank.hp - projectile.weapon.damage)
        projectile.damaged_targets.add(tank.player_id)

    def _handle_projectile_projectile_contact(
        self,
        projectile_a: ProjectileState,
        projectile_b: ProjectileState,
    ):
        normal = projectile_b.body_np.getPos() - projectile_a.body_np.getPos()
        normal.y = 0.0
        if normal.length_squared() <= 1e-9:
            normal = Vec3(1.0, 0.0, 0.0)
        apply_elastic_contact_impulse(
            projectile_a.body_np.node(),
            projectile_b.body_np.node(),
            normal,
            PROJECTILE_PROJECTILE_RESTITUTION,
        )

    def _projectile_ground_impact_speed(self, projectile: ProjectileState) -> float:
        surface_normal = surface_normal_at_x(self.terrain, projectile.body_np.getX())
        velocity = projectile.previous_velocity
        return max(0.0, -velocity.dot(surface_normal))

    def _reactivate_nearby_bodies(self, crater_x: float, radius: float):
        for tank in self.tanks:
            if abs(tank.body_np.getX() - crater_x) <= radius:
                tank.body_np.node().setActive(True)
        for projectile in self.projectiles:
            if abs(projectile.body_np.getX() - crater_x) <= radius:
                projectile.body_np.node().setActive(True)

    def _check_win_state(self):
        losers = [
            tank
            for tank in self.tanks
            if tank.hp <= 0 or tank.body_np.getZ() < self.terrain.kill_z
        ]
        if not losers:
            return
        self.match_over = True
        if len(losers) == 2:
            self.winner_text = "Draw! Press R to restart"
        else:
            winner_id = 1 if losers[0].player_id == 2 else 2
            self.winner_text = f"{PLAYER_NAMES[winner_id]} wins! Press R to restart"
        for tank in self.tanks:
            tank.body_np.node().setLinearVelocity(Vec3(0.0, 0.0, 0.0))
            tank.body_np.node().setAngularVelocity(Vec3(0.0, 0.0, 0.0))
        for projectile in list(self.projectiles):
            remove_projectile(self.world, projectile)
        self.projectiles.clear()

    def _refresh_hud(self):
        if len(self.tanks) == 2:
            self.hud.update(self.tanks[0], self.tanks[1], self.winner_text)

    def _clear_world(self):
        if not self.world:
            return

        for projectile in list(self.projectiles):
            remove_projectile(self.world, projectile)
        self.projectiles.clear()

        for tank in self.tanks:
            self.world.removeRigidBody(tank.body_np.node())
            tank.body_np.removeNode()
        self.tanks.clear()

        if self.terrain:
            self.world.removeRigidBody(self.terrain.terrain_body_np.node())
            self.terrain.terrain_body_np.removeNode()
            if self.terrain.profile_visual_np:
                self.terrain.profile_visual_np.removeNode()
            self.terrain.terrain_root.removeNode()
            self.terrain = None

        if self.debug_np:
            self.debug_np.removeNode()
            self.debug_np = None

        self.world = None

    def _pressed_once(self, input_name: str) -> bool:
        now = self._is_key_down(input_name)
        was = self.button_states.get(input_name, False)
        self.button_states[input_name] = now
        return now and not was

    def _bind_key(self, key: str):
        if key in self.key_state:
            return
        self.key_state[key] = False
        self.accept(key, self._set_key_state, [key, True])
        self.accept(f"{key}-up", self._set_key_state, [key, False])

    def _set_key_state(self, key: str, is_down: bool):
        self.key_state[key] = is_down

    def _is_key_down(self, key: str) -> bool:
        return self.key_state.get(key, False)

    def _apply_horizontal_movement(self, tank: TankState, move_dir: int, dt: float):
        body = tank.body_np.node()
        velocity = body.getLinearVelocity()
        current_x = velocity.x

        if (
            tank.grounded
            and move_dir != 0
            and abs(tank.body_np.getX()) <= self.terrain.world_width / 2.0 + 1.0
        ):
            current_x = tank.body_np.getX()
            desired_x = current_x + move_dir * TANK_MOVE_SPEED * dt
            desired_surface = self._support_surface_height(desired_x)
            if desired_surface is not None:
                target_z = desired_surface + TANK_HALF_HEIGHT + TANK_RIDE_HEIGHT
                smooth_factor = min(1.0, TANK_SURFACE_SMOOTHING * dt)
                current_z = tank.body_np.getZ()
                smoothed_z = current_z + (target_z - current_z) * smooth_factor
                tank.body_np.setPos(desired_x, 0.0, smoothed_z)
                tank.body_np.setR(0.0)
                self._align_tank_to_surface(tank, desired_x)
                velocity.x = move_dir * TANK_MOVE_SPEED
                velocity.y = 0.0
                velocity.z = min(velocity.z, 0.0)
                body.setLinearVelocity(velocity)
                body.setAngularVelocity(Vec3(0.0, 0.0, 0.0))
                body.setActive(True)
                return
            tank.grounded = False
            tank.airborne = True
            tank.support_frames = 0

        if move_dir != 0:
            target_x = move_dir * TANK_MOVE_SPEED
            accel = TANK_GROUND_ACCEL if tank.grounded else TANK_AIR_ACCEL
        else:
            target_x = 0.0
            accel = TANK_IDLE_DECEL if tank.grounded else TANK_AIR_ACCEL * 0.35

        max_delta = accel * dt
        delta = target_x - current_x
        if abs(delta) <= max_delta:
            velocity.x = target_x
        else:
            velocity.x = current_x + max_delta * (1.0 if delta > 0.0 else -1.0)

        body.setLinearVelocity(velocity)
        if move_dir != 0:
            body.setActive(True)

    def _apply_knockback_motion(self, tank: TankState, dt: float):
        body = tank.body_np.node()
        desired_x = tank.body_np.getX() + tank.knockback_velocity_x * dt
        surface_height = self._support_surface_height(desired_x)

        if tank.grounded and surface_height is not None:
            target_z = surface_height + TANK_HALF_HEIGHT + TANK_RIDE_HEIGHT
            current_z = tank.body_np.getZ()
            smooth_factor = min(1.0, TANK_SURFACE_SMOOTHING * dt)
            smoothed_z = current_z + (target_z - current_z) * smooth_factor
            tank.body_np.setPos(desired_x, 0.0, smoothed_z)
            tank.body_np.setR(0.0)
            self._align_tank_to_surface(tank, desired_x)
            velocity = body.getLinearVelocity()
            velocity.x = tank.knockback_velocity_x
            velocity.y = 0.0
            velocity.z = min(velocity.z, 0.0)
            body.setLinearVelocity(velocity)
            body.setAngularVelocity(Vec3(0.0, 0.0, 0.0))
        else:
            if surface_height is None:
                tank.grounded = False
                tank.airborne = True
                tank.support_frames = 0
            velocity = body.getLinearVelocity()
            velocity.x = tank.knockback_velocity_x
            body.setLinearVelocity(velocity)

        body.setActive(True)
        tank.knockback_velocity_x *= max(0.0, 1.0 - TANK_KNOCKBACK_DECAY * dt)

    def _resolve_tank_tank_collision(self):
        if len(self.tanks) != 2:
            return

        tank_a, tank_b = self.tanks
        pos_a = tank_a.body_np.getPos()
        pos_b = tank_b.body_np.getPos()
        dx = pos_b.x - pos_a.x
        vel_a = tank_a.body_np.node().getLinearVelocity()
        vel_b = tank_b.body_np.node().getLinearVelocity()
        relative_vx = vel_b.x - vel_a.x
        closing = dx * relative_vx < 0.0
        if not self._lower_hulls_intersect(tank_a, tank_b, closing):
            return

        direction = 1.0 if dx >= 0.0 else -1.0
        half_width_a, _ = self._lower_hull_extents(tank_a)
        half_width_b, _ = self._lower_hull_extents(tank_b)
        min_sep = half_width_a + half_width_b + TANK_COLLISION_BUFFER
        overlap = min_sep - abs(dx)
        separation = max(0.0, overlap) * 0.5 + 0.02

        new_ax = pos_a.x - direction * separation
        new_bx = pos_b.x + direction * separation
        tank_a.body_np.setX(new_ax)
        tank_b.body_np.setX(new_bx)
        surface_a = self._support_surface_height(new_ax)
        surface_b = self._support_surface_height(new_bx)
        if surface_a is not None:
            tank_a.body_np.setZ(surface_a + TANK_HALF_HEIGHT + TANK_RIDE_HEIGHT)
        if surface_b is not None:
            tank_b.body_np.setZ(surface_b + TANK_HALF_HEIGHT + TANK_RIDE_HEIGHT)
        tank_a.body_np.setR(0.0)
        tank_b.body_np.setR(0.0)
        if surface_a is not None:
            self._align_tank_to_surface(tank_a, new_ax)
        if surface_b is not None:
            self._align_tank_to_surface(tank_b, new_bx)

        u1 = vel_a.x
        u2 = vel_b.x
        e = TANK_CONTACT_RESTITUTION
        v1 = ((1.0 - e) * u1 + (1.0 + e) * u2) * 0.5
        v2 = ((1.0 + e) * u1 + (1.0 - e) * u2) * 0.5
        if abs(v1 - v2) < 0.15:
            v1 = -direction * 2.8
            v2 = direction * 2.8
        vel_a.x = v1
        vel_b.x = v2
        vel_a.y = 0.0
        vel_b.y = 0.0
        tank_a.body_np.node().setLinearVelocity(vel_a)
        tank_b.body_np.node().setLinearVelocity(vel_b)
        tank_a.body_np.node().setAngularVelocity(Vec3(0.0, 0.0, 0.0))
        tank_b.body_np.node().setAngularVelocity(Vec3(0.0, 0.0, 0.0))
        tank_a.body_np.node().setActive(True)
        tank_b.body_np.node().setActive(True)

    def _support_surface_height(self, world_x: float) -> float | None:
        if not self._has_support_at(world_x):
            return None
        probe = TANK_HALF_WIDTH * TANK_SUPPORT_PROBE
        left = sample_surface_height(self.terrain, world_x - probe)
        center = sample_surface_height(self.terrain, world_x)
        right = sample_surface_height(self.terrain, world_x + probe)
        return 0.2 * left + 0.6 * center + 0.2 * right

    def _align_tank_to_surface(self, tank: TankState, world_x: float):
        if not self._has_support_at(world_x):
            return
        tank.visual_np.setR(surface_angle_degrees(self.terrain, world_x))

    @staticmethod
    def _lower_hull_extents(tank: TankState) -> tuple[float, float]:
        angle = radians(abs(tank.visual_np.getR()))
        half_width = TANK_HALF_WIDTH * cos(angle) + TANK_LOWER_HALF_HEIGHT * sin(angle)
        half_height = TANK_LOWER_HALF_HEIGHT * cos(angle) + TANK_HALF_WIDTH * sin(angle)
        return half_width, half_height

    def _lower_hulls_intersect(self, tank_a: TankState, tank_b: TankState, closing: bool) -> bool:
        early_margin = 0.0
        if closing:
            speed = abs(tank_b.body_np.node().getLinearVelocity().x - tank_a.body_np.node().getLinearVelocity().x)
            early_margin = max(0.10, min(0.28, speed * FIXED_DT * 1.8))

        rect_a = self._lower_hull_rect(tank_a, TANK_COLLISION_BUFFER * 0.5 + early_margin)
        rect_b = self._lower_hull_rect(tank_b, TANK_COLLISION_BUFFER * 0.5 + early_margin)
        delta_x = rect_b["center"][0] - rect_a["center"][0]
        if abs(delta_x) > rect_a["half_width"] + rect_b["half_width"] + TANK_COLLISION_BUFFER + early_margin * 2.0:
            return False

        for axis in rect_a["axes"] + rect_b["axes"]:
            distance = abs(self._dot_2d(rect_b["center"], axis) - self._dot_2d(rect_a["center"], axis))
            radius_a = self._project_rect_radius(rect_a, axis)
            radius_b = self._project_rect_radius(rect_b, axis)
            if distance > radius_a + radius_b:
                return False
        return True

    def _lower_hull_rect(self, tank: TankState, extra_width: float) -> dict[str, object]:
        angle = radians(tank.visual_np.getR())
        axis_width = (cos(angle), sin(angle))
        axis_height = (-sin(angle), cos(angle))
        return {
            "center": (tank.body_np.getX(), tank.body_np.getZ()),
            "axes": (axis_width, axis_height),
            "half_width": TANK_HALF_WIDTH + extra_width,
            "half_height": TANK_LOWER_HALF_HEIGHT + 0.03,
        }

    @staticmethod
    def _project_rect_radius(rect: dict[str, object], axis: tuple[float, float]) -> float:
        axis_width, axis_height = rect["axes"]
        return (
            rect["half_width"] * abs(TanksHotseatGame._dot_2d(axis_width, axis))
            + rect["half_height"] * abs(TanksHotseatGame._dot_2d(axis_height, axis))
        )

    @staticmethod
    def _dot_2d(a: tuple[float, float], b: tuple[float, float]) -> float:
        return a[0] * b[0] + a[1] * b[1]

    def _has_support_at(self, world_x: float) -> bool:
        limit = self.terrain.world_width * 0.5
        return abs(world_x) <= limit

    @staticmethod
    def _pair_key(node_a, node_b) -> tuple[int, int]:
        a_id = TanksHotseatGame._node_key(node_a)
        b_id = TanksHotseatGame._node_key(node_b)
        return (a_id, b_id) if a_id <= b_id else (b_id, a_id)

    @staticmethod
    def _node_key(node) -> int:
        return int(node.this)


def main():
    game = TanksHotseatGame()
    game.run()
