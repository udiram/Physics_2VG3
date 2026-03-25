# Exercise 5

This exercise covers:

- scenario `5`: stable five-box stack on an inclined plane
- scenario `6`: the same stack in a moving frame, with everything translating left at speed `-1`

The physical setup was kept fixed as required:

- `sin(angle) = 0.1`
- `e = 0.05`
- `mu_s = mu_k = 0.2`
- gravity on

## Chosen Parameters

After tuning the solver parameters around the already-working stack setup, I used:

- `iterations = 120`
- `bias_beta = 0.15`
- `bias_slop = 0.005`

These gave the best overall result in the stable region I tested:

- the stack remains intact,
- visible jitter is low,
- squeezing is small,
- and scenario 6 matches scenario 5 closely in the plane's own frame.

The long-run summary is in [`output/exercise5/stack_summary.json`](output/exercise5/stack_summary.json).

## Scenario 5: Stable Stack On Incline

With the chosen parameters, the five boxes settle into a stable off-center stack and remain stacked over an 8 second run. The final image shows the intended lecture-style arrangement: large red block at the base, then black, green, magenta, and white above it with slight offsets rather than a perfectly centered tower.

Representative screenshot:

- [`output/exercise5/scenario5.png`](output/exercise5/scenario5.png)

At `t = 8.0 s`, the stack is still in contact and still quiet:

- contact constraints active: `10`
- relative velocities are all about `0.00393` in `z`
- the largest relative angle error from the plane is about `0.00751 rad` which is about `0.43 degrees`

Final relative body configuration with respect to the plane:

| Box | Relative x | Relative z | Relative angle (rad) |
| --- | ---: | ---: | ---: |
| body 1 | -0.22881 | 2.53611 | 0.00001 |
| body 2 | -0.19110 | 3.31614 | 0.00112 |
| body 3 | -0.16705 | 3.86566 | 0.00173 |
| body 4 | -0.17625 | 4.39717 | -0.00281 |
| body 5 | -0.21507 | 4.91296 | 0.00751 |

So the stack is stable, slightly off center, and not showing large jitter or collapse.

## Scenario 6: Moving Target

Scenario 6 repeats the same physical setup but with everything moving left at speed `-1`. The expected result is that the stack should behave the same in that moving frame.

Representative screenshot:

- [`output/exercise5/scenario6.png`](output/exercise5/scenario6.png)

That is exactly what happens visually: the stack looks the same as scenario 5, except the whole system has translated left.

I compared the final stack geometry in the plane's own frame for scenarios 5 and 6. The summed mismatch over all five boxes, using relative x, relative z, and relative angle, was:

- moving-frame mismatch: `0.00843`

That is very small. The relative positions and angles agree closely enough that the two stacks are effectively the same physical state, which is what we want for the Galilean invariance check.

## Conclusion

Exercise 5 is working with:

- `iterations = 120`
- `bias_beta = 0.15`
- `bias_slop = 0.005`

Scenario 5 gives a stable off-center stack on the incline, and scenario 6 gives the same stack in the moving frame. The residual motion is small, the jitter is low, and the moving-frame result closely matches the stationary one.
