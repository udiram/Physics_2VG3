# HW4 Usage and Interpretation Guide

## What I built
The assignment is implemented in `HW4/rigidbody2d_collision.py` with a full square-square rigid body collision solver (no hack assumptions), all required scenarios (`0..4`), and both restitution settings (`e=1` and `e=0`).

I also added:
- `HW4/tests/test_rigidbody2d_collision.py` for automated checks
- `HW4/output/hw4_results.json` with full raw results
- `HW4/output/hw4_summary.csv` with a compact numeric summary
- `HW4/HW4_answers.pdf` as the written hand-in document

## How to run

### Graphical mode (Panda3D)
Run one scenario in a Panda3D window:

```bash
./.venv/bin/python HW4/rigidbody2d_collision.py --scenario 0 --restitution 1.0 --max-time 6.0
```

Use `--scenario 0..4` and switch `--restitution` between `1.0` and `0.0`.

### Headless single case

```bash
./.venv/bin/python HW4/rigidbody2d_collision.py --headless --scenario 2 --restitution 0.0 --max-time 6.0 --stop-after-first-collision
```

### Run all assignment cases and regenerate report files

```bash
./.venv/bin/python HW4/rigidbody2d_collision.py \
  --run-all \
  --dt 0.004166666666666667 \
  --max-time 6.0 \
  --stop-after-first-collision \
  --write-json \
  --write-csv \
  --generate-answers-pdf
```

## How to read results for each assignment section

### Exercise 1
Look at `HW4/rigidbody2d_collision.py`, especially `collide_square_square(...)` and the impulse resolution path. This is the completed general collision implementation.

### Exercise 2(a): `e=1`
Use the "Exercise 2 (a)" section in `HW4/HW4_answers.pdf`.

For each scenario, it reports:
- collision time
- averaged collision point (3D)
- collision normal
- whether contact was single or simultaneous
- full contact set before averaging, including corner/face classification

### Exercise 2(b): `e=0`
Use the "Exercise 2 (b)" section in `HW4/HW4_answers.pdf`.

Per the instructions, this section reports first-contact properties for each scenario.

### Exercise 3
Use the "Exercise 3" section in `HW4/HW4_answers.pdf` and the numeric values in `HW4/output/hw4_summary.csv`.

For each scenario and restitution value, compare pre- vs post-collision:
- total momentum
- total angular momentum
- translational kinetic energy
- rotational kinetic energy
- total kinetic energy

## Test suite
Run:

```bash
./.venv/bin/pytest -q HW4/tests/test_rigidbody2d_collision.py
```

Current status: `5 passed`.
