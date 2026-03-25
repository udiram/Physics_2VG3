# Exercise 2

This write-up uses the current `rigidbody2d_seq.py` with:

- `dt = 1/120`
- `bias_beta = 0.15`
- `bias_slop = 0.005`
- Scenario 0: incline with `sin(angle)=0.1`, `e=0.05`, `mu_s=mu_k=0.5`, gravity on
- Scenario 1: flat plane, `e=0.0`, `mu_s=mu_k=0.5`, gravity off, initial velocity `(0,0,-1)`

Raw sweep data is in [`output/exercise2/iteration_sweep.json`](output/exercise2/iteration_sweep.json).

## Scenario 0

The box first contacts the incline at about `t = 0.35 s`. With enough iterations it hits the plane, rotates slightly to match the ramp, and then settles into a nearly static resting contact. The final visual result is the expected one: the block stays on the incline instead of sliding away.

Selected results after 4 seconds:

| Iterations | Final speed | Final tangent speed | Max upward `v_z` after impact | Observation |
| --- | ---: | ---: | ---: | --- |
| 1 | 0.03536 | -0.03418 | 0.70305 | Clearly wrong. The block gets a noticeable upward kick, keeps rotating, and slides down the ramp. |
| 2 | 0.00734 | 0.00442 | 1.03948 | Still wrong. The solver injects a strong nonphysical bounce and the block drifts upslope. |
| 4 | 0.00160 | 0.00152 | 0.02995 | First iteration count that looks acceptable. The box settles with only tiny residual creep. |
| 8 | 0.00380 | 0.00025 | 0.21421 | Visually stable. Residual motion is dominated by the bias term rather than sliding. |
| 30 | 0.00395 | 0.00000 | 0.22300 | Converged reference. Further iterations do not change the result visibly. |
| 120 | 0.00395 | 0.00000 | 0.22300 | Same visual outcome as 30. |

What I see:

- At 1 to 2 iterations, the square does not satisfy both corner contacts and friction consistently.
- The first few impulses create the wrong combination of translation and rotation, so the block either hops or creeps along the incline instead of settling.
- From about 4 iterations onward, the motion looks qualitatively correct.
- From about 30 iterations onward, the result is visually converged.
- The tiny remaining normal motion at higher iteration counts comes from the velocity bias used to keep resting contacts separated, not from a visible bounce.

Representative screenshots:

- Bad low-iteration case: [`output/exercise2/scenario0_iter1.png`](output/exercise2/scenario0_iter1.png)
- First acceptable case: [`output/exercise2/scenario0_iter4.png`](output/exercise2/scenario0_iter4.png)

## Scenario 1

The box first contacts the plane at about `t = 1.0083 s`. Because `e=0` and gravity is off, the physically correct result is that it should stop immediately and remain at rest on the plane.

Selected results after 4 seconds:

| Iterations | Final speed | Final tangent speed | Final normal speed | Observation |
| --- | ---: | ---: | ---: | --- |
| 1 | 0.23316 | -0.09320 | 0.21372 | Wrong. The box rebounds upward and also picks up sideways drift and rotation. |
| 2 | 0.19703 | 0.00960 | 0.19680 | Still wrong. It keeps floating upward even though `e=0`. |
| 4 | 0.00007 | 0.00006 | 0.00002 | Good. The box essentially stops on contact. |
| 8 | 0.00003 | -0.00003 | 0.00001 | Very good. Residual motion is negligible. |
| 30 | 0.00000 | 0.00000 | 0.00000 | Converged. |
| 120 | 0.00000 | 0.00000 | 0.00000 | Same as 30 within numerical precision. |

What I see:

- At 1 to 2 iterations, the two face contacts are under-resolved.
- That leaves unbalanced angular and linear error, so the block leaves the plane even though it should stop.
- At 4 iterations the collision is already essentially correct.
- By 8 to 30 iterations the result is numerically very close to exact rest.

Representative screenshots:

- Bad low-iteration case: [`output/exercise2/scenario1_iter1.png`](output/exercise2/scenario1_iter1.png)
- Good case: [`output/exercise2/scenario1_iter4.png`](output/exercise2/scenario1_iter4.png)

## Recommended Iteration Count

For these Exercise 2 tests:

- The minimum iteration count I would call acceptable is `4`.
- The first count I would call comfortably converged is about `30`.
- For an actual demo or submission run, `30` to `60` iterations is a good choice because it removes the low-iteration artifacts without changing the result visibly beyond that.

## What Goes Wrong For Small Numbers Of Iterations

Sequential impulses solve one contact at a time. A square hitting a plane usually creates two contact constraints. With too few iterations:

- the first contact correction changes the body's translation and rotation,
- that change makes the second contact incorrect again,
- and the solver stops before those constraints have been brought into agreement.

That produces exactly the artifacts seen here:

- nonphysical bounce even when restitution is zero,
- sideways drift on a case that should have no lateral motion,
- extra rotation,
- and failure to settle cleanly on the incline.

With more iterations, the solver repeatedly revisits the same constraints and converges toward the correct coupled solution.
