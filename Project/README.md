# Tanks Hotseat Submission

This folder contains the source-code submission for the Tanks Hotseat class project.

## What This Is

Tanks Hotseat is a two-player hotseat artillery game. In this context, hotseat means two people play locally on the same computer using the same keyboard. Each player controls one tank and tries to destroy the other tank or force it off the map.

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

## Goal

- Reduce the opposing tank to `0 HP`, or
- force the opposing tank to fall off the edge of the terrain

## Included Files

- `main.py`: game entrypoint
- `requirements.txt`: dependency list
- `assets/`: local model assets used by the game
- `src/tanks_game/`: game source files
- `Assignment_Writeup.pdf`: compiled assignment write-up
