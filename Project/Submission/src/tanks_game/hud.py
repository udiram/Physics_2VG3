from __future__ import annotations

from direct.gui.OnscreenText import OnscreenText
from panda3d.core import TextNode

from .combat import cooldown_remaining


class GameHud:
    def __init__(self):
        self.left_text = OnscreenText(
            text="",
            pos=(-1.30, 0.92),
            scale=0.055,
            align=TextNode.ALeft,
            fg=(1, 1, 1, 1),
            mayChange=True,
        )
        self.right_text = OnscreenText(
            text="",
            pos=(1.30, 0.92),
            scale=0.055,
            align=TextNode.ARight,
            fg=(1, 1, 1, 1),
            mayChange=True,
        )
        self.banner_text = OnscreenText(
            text="",
            pos=(0.0, 0.0),
            scale=0.11,
            align=TextNode.ACenter,
            fg=(1.0, 0.98, 0.84, 1.0),
            shadow=(0, 0, 0, 0.55),
            mayChange=True,
        )
        self.footer_text = OnscreenText(
            text="R: restart    Esc: quit    F3: debug",
            pos=(0.0, -0.97),
            scale=0.038,
            align=TextNode.ACenter,
            fg=(0.95, 0.95, 0.95, 1.0),
        )
        self.left_controls = OnscreenText(
            text="P1 Controls\nA/D move  W/S aim\nQ rapid  E heavy  LShift jump",
            pos=(-1.30, -0.80),
            scale=0.038,
            align=TextNode.ALeft,
            fg=(0.92, 0.96, 0.92, 1.0),
            mayChange=False,
        )
        self.right_controls = OnscreenText(
            text="P2 Controls\nJ/L move  I/K aim\nU rapid  O heavy  RShift jump",
            pos=(1.30, -0.80),
            scale=0.038,
            align=TextNode.ARight,
            fg=(0.92, 0.96, 0.92, 1.0),
            mayChange=False,
        )

    def update(self, left_tank, right_tank, winner_text: str | None = None):
        self.left_text.setText(self._tank_status(left_tank))
        self.right_text.setText(self._tank_status(right_tank))
        self.banner_text.setText(winner_text or "")

    def destroy(self):
        self.left_text.destroy()
        self.right_text.destroy()
        self.banner_text.destroy()
        self.footer_text.destroy()
        self.left_controls.destroy()
        self.right_controls.destroy()

    def _tank_status(self, tank) -> str:
        rapid = self._cooldown_text(tank, "rapid")
        heavy = self._cooldown_text(tank, "heavy")
        airborne = "YES" if tank.airborne else "NO"
        return (
            f"P{tank.player_id}  HP: {tank.hp}\n"
            f"Angle: {tank.turret_pitch_deg:.0f} deg\n"
            f"Rapid: {rapid}\n"
            f"Heavy: {heavy}\n"
            f"In Air: {airborne}"
        )

    @staticmethod
    def _cooldown_text(tank, weapon_name: str) -> str:
        remaining = cooldown_remaining(tank.cooldowns, weapon_name)
        return "READY" if remaining <= 0.0 else f"{remaining:0.1f}s"
