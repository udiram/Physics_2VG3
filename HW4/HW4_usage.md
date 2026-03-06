# HW4 Usage and Interpretation Guide

## What was implemented
- `HW4/rigidbody2d_collision.py`
  - Full 2D rigid-body square collision detection/resolution (no hack assumptions).
  - SAT overlap test + explicit contact extraction.
  - Contact averaging for simultaneous contacts, as required.
  - Scenario selector `scenario = 0..4` exactly matching the assignment.
  - Coefficient of restitution support (`e=1` and `e=0`).
  - Both graphical Panda3D mode and deterministic headless batch mode.
- `HW4/tests/test_rigidbody2d_collision.py`
  - Automated validation of conservation laws, scenario coverage, and contact averaging logic.
- Generated artifacts:
  - `HW4/output/hw4_results.json`
  - `HW4/output/hw4_summary.csv`
  - `HW4/HW4_answers.pdf`

## How to run

### 1. Graphical simulation (Panda3D window)
Run one scenario interactively:

```bash
./.venv/bin/python HW4/rigidbody2d_collision.py --scenario 0 --restitution 1.0 --max-time 6.0
```

- Change `--scenario` to `0,1,2,3,4`.
- Use `--restitution 1.0` for Exercise 2(a), `--restitution 0.0` for Exercise 2(b).

### 2. Headless single-case run

```bash
./.venv/bin/python HW4/rigidbody2d_collision.py --headless --scenario 2 --restitution 0.0 --max-time 6.0 --stop-after-first-collision
```

### 3. Run all required assignment cases and build report outputs

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

## How to interpret outputs per assignment section

## Exercise 1 (code completion)
- Main implementation is in `HW4/rigidbody2d_collision.py`.
- Function `collide_square_square(...)` performs general square-square collision detection.
- Collision resolution includes linear and angular impulse updates.

## Exercise 2(a), e=1
Use either:
- `HW4/HW4_answers.pdf` section "Exercise 2 (a)", or
- `HW4/output/hw4_results.json` filtered by `"restitution": 1.0`.

For each scenario, read:
- `time`
- `point` (3D collision location)
- `normal` (collision normal)
- `contact_mode` and `num_contacts_before_average`
- `contact_details` (all simultaneous contacts before averaging)

## Exercise 2(b), e=0
Use either:
- `HW4/HW4_answers.pdf` section "Exercise 2 (b)", or
- `HW4/output/hw4_results.json` filtered by `"restitution": 0.0`.

Per assignment instructions, only first-contact properties are required and reported.

## Exercise 3 (momentum, angular momentum, energy)
Use:
- `HW4/HW4_answers.pdf` section "Exercise 3"
- `HW4/output/hw4_summary.csv` (quick numerical table)

For each scenario and restitution:
- "pre" and "post" values are immediately before/after the first collision.
- Momentum and angular momentum conservation can be checked from deltas.
- Energy behavior:
  - `e=1`: total kinetic energy approximately conserved.
  - `e=0`: total kinetic energy decreases; translation/rotation partition changes.

## Test suite
Run:

```bash
./.venv/bin/pytest -q HW4/tests/test_rigidbody2d_collision.py
```

Current result: `5 passed`.
