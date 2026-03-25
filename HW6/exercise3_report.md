# Exercise 3

This write-up uses the same solver settings chosen from Exercise 2:

- `iterations = 30`
- `dt = 1/120`
- `bias_beta = 0.15`
- `bias_slop = 0.005`

Raw energy measurements are in [`output/exercise3/energy_summary.json`](output/exercise3/energy_summary.json).

## Scenario 2: Bounce On Incline

Setup:

- Inclined plane with `sin(angle) = 0.1`
- `e = 1.0`
- Gravity off
- Initial velocity toward the plane: approximately `(-0.1, 0, -0.994987)`

What I see:

- The block hits the incline and reflects back along the same line it came in on.
- The normal component of velocity flips sign from about `-1.0` to `+1.0`.
- The tangential component stays essentially zero.
- The block does not pick up visible spin.
- After impact it moves away cleanly from the ramp, exactly as expected for an elastic collision with no gravity.

Measured values:

| Quantity | Before collision | After collision |
| --- | ---: | ---: |
| Time | 0.95833 s | 0.97500 s |
| Velocity x | -0.10000000 | 0.09999995 |
| Velocity z | -0.99498743 | 0.99498743 |
| Tangential velocity | -2.37e-09 | -4.95e-08 |
| Normal velocity | -0.99999999 | 0.99999999 |
| Translational energy | 0.5 | 0.5 |
| Rotational energy | 0.0 | 1.83e-17 |
| Total energy | 0.5 | 0.5 |

Energy change:

- Absolute change: `0.0`
- Relative change: `0.0`

Representative render:

- [`output/exercise3/scenario2.png`](output/exercise3/scenario2.png)

## Scenario 3: Bounce On Flat Plane

Setup:

- Flat plane
- `e = 1.0`
- Gravity off
- Initial velocity straight down: `(0, 0, -1)`

What I see:

- The block hits the plane and comes straight back up.
- The vertical speed is unchanged in magnitude.
- There is essentially no sideways drift.
- There is no visible rotation after impact.
- This is the cleanest case and it behaves almost exactly like the ideal textbook reflection.

Measured values:

| Quantity | Before collision | After collision |
| --- | ---: | ---: |
| Time | 1.00000 s | 1.01667 s |
| Velocity x | 0.0 | -5.96e-08 |
| Velocity z | -1.0 | 1.0 |
| Tangential velocity | 0.0 | -5.96e-08 |
| Normal velocity | -1.0 | 1.0 |
| Translational energy | 0.5 | 0.5 |
| Rotational energy | 0.0 | 1.94e-17 |
| Total energy | 0.5 | 0.5 |

Energy change:

- Absolute change: `0.0`
- Relative change: `0.0`

Representative render:

- [`output/exercise3/scenario3.png`](output/exercise3/scenario3.png)

## Discussion

With `30` iterations, both elastic tests behave correctly:

- scenario 2 reflects about the plane normal, so the block returns exactly the way it came in,
- scenario 3 reverses only the vertical component and goes straight back up,
- and total kinetic energy is conserved to numerical precision.

The only nonzero post-collision errors are tiny floating-point scale terms:

- tangential velocity is around `10^-8`,
- rotational energy is around `10^-17`.

These are far below anything visible in the render, so for practical purposes the collisions are perfectly elastic.
