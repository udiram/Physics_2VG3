# Tanks Hotseat

Two-player same-keyboard artillery game built with Panda3D and Bullet Physics for a 2VG3 class project.

## Overview

Two tanks fight on procedurally generated hills in a 2D side-view match. Players move left and right, aim their turrets, jump, and fire two shell types. Terrain is destructible, shells can bounce and roll, and a tank loses by running out of HP or falling off the edge of the map.

## Requirements

- Windows
- Python 3.12

## Setup

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

## Run

```powershell
python main.py
```

## Tests

```powershell
python -m unittest discover -s tests -p "test_*.py"
```

## Controls

### Player 1

- `A` / `D`: move left / right
- `W` / `S`: aim turret up / down
- `Q`: rapid fire
- `E`: heavy fire
- `Left Shift`: jump

### Player 2

- `J` / `L`: move left / right
- `I` / `K`: aim turret up / down
- `U`: rapid fire
- `O`: heavy fire
- `Right Shift`: jump

### Global

- `R`: restart with new random terrain
- `Esc`: quit
- `F3`: toggle Bullet debug view

## Match Rules

- Each tank starts with `100 HP`.
- `Rapid fire` deals `5` damage and has a `0.5s` cooldown.
- `Heavy fire` deals `25` damage and has a `3.0s` cooldown.
- Shells can bounce off terrain and other shells.
- Ground impacts can carve craters into the terrain.
- A tank loses if its HP reaches `0` or if it falls below the terrain kill plane.

## Current Features

- Orthographic 2D side-view presentation
- Procedural finite hill generation
- Destructible terrain with live collision rebuilds
- Same-keyboard 2-player combat
- Turret aiming with on-screen angle display
- Rapid and heavy projectile types
- Tank jump
- Tank-vs-tank elastic collision handling
- Projectile-vs-tank knockback and damage
- Projectile bounce and rolling behavior
- HUD with HP, cooldowns, controls, and win banner

## Project Structure

- `main.py`: entrypoint
- `requirements.txt`: Python dependency list
- `src/tanks_game/config.py`: gameplay constants and weapon specs
- `src/tanks_game/terrain.py`: terrain generation and crater deformation
- `src/tanks_game/combat.py`: tank and projectile creation/state helpers
- `src/tanks_game/hud.py`: HUD rendering
- `src/tanks_game/game.py`: main Panda3D game loop
- `tests/`: small automated logic checks
- `writeup_answers.md`: brief written responses for the class exercises
