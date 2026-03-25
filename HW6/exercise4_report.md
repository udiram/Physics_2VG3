# Exercise 4

This exercise uses scenario `4`:

- flat plane
- gravity on
- `e = 1.0`
- no friction

The main result is that the block keeps bouncing and does not settle, which is the expected qualitative behavior. I used:

- `iterations = 30`
- `dt = 1/120`
- `bias_beta = 0.15`
- `bias_slop = 0.005`

Raw measurements are in [`output/exercise4/energy_summary.json`](output/exercise4/energy_summary.json).

## Base Behavior

With the current solver settings, the box bounces repeatedly and keeps nearly the same height on every cycle. There is essentially no spin because friction is zero and the contact is symmetric.

Representative render:

- [`output/exercise4/scenario4.png`](output/exercise4/scenario4.png)

Measured apex heights over 6 seconds:

| Bounce apex time | Height `z` | Total mechanical energy |
| --- | ---: | ---: |
| 0.8250 s | 0.55382 | 5.54167 |
| 1.7333 s | 0.55486 | 5.54861 |
| 2.6583 s | 0.55451 | 5.54861 |
| 3.5667 s | 0.55556 | 5.55556 |
| 4.4917 s | 0.55521 | 5.55556 |
| 5.4000 s | 0.55625 | 5.56250 |

This is very close to a constant-bounce-height motion, but not perfectly energy-conserving. Over the 6 second run:

- initial total mechanical energy: `5.5`
- final total mechanical energy: `5.70139`
- energy span over the run: `0.40278`

So the motion is qualitatively correct, but there is still a small upward energy drift. I also checked iterations `4, 8, 30, 60, 120`; beyond about `8` iterations, the energy drift is basically unchanged. That means the remaining error is mainly from finite timestep and discrete contact handling, not from too few solver iterations.

## Conservation Of Energy

The total mechanical energy here is:

- translational kinetic energy
- rotational kinetic energy
- gravitational potential energy `m g z`

Because friction is zero and `e = 1.0`, the ideal result would be constant total mechanical energy. In the current simulation the total energy is close to constant, but not exact. The bounce height drifts upward slightly from about `z = 0.5538` to `z = 0.5563` over several cycles.

So my conclusion is:

- energy is approximately conserved,
- the block does bounce effectively forever,
- but there is a small numerical energy gain over time.

## Impact Of Velocity Bias

There are two slightly different answers here depending on what is meant by “velocity bias”.

### Current Solver

In the current solver, elastic impacts do **not** receive the bias correction in the usual repeated-bounce regime, because the code only applies bias for low-speed resting-style contacts. I verified this by comparing:

- current solver with `bias_on_elastic = False`
- current solver with `bias_on_elastic = True`

Those two runs gave the same energy history for scenario 4. So in the current guarded implementation, the practical impact of velocity bias on this exercise is essentially none.

### Classic Elastic-Bias Demo

To answer the homework question directly, I also forced the classic bias behavior on elastic impacts by overriding the elastic-bias threshold in a diagnostic run. That produces the expected bad result: the bias injects energy into the bounce.

Forced elastic bias results:

| Case | Initial energy | Final energy | Example apex heights |
| --- | ---: | ---: | --- |
| Forced bias, `beta = 0.15` | 5.5 | 16.74972 | 0.68392, 0.98752, 1.13844, 1.27407, 1.69857 |
| Forced bias, `beta = 0.30` | 5.5 | 42.55890 | 0.82152, 0.98504, 1.98431, 3.66011 |

Representative forced-bias render:

- [`output/exercise4/scenario4_forced_bias.png`](output/exercise4/scenario4_forced_bias.png)

This shows the real danger of velocity bias in elastic repeated collisions:

- it pushes the bodies apart every impact,
- that adds energy instead of only correcting overlap,
- and the bounce height grows rapidly.

The larger the bias factor `beta`, the worse the energy blow-up becomes.

## Conclusion

Scenario 4 works qualitatively: the square keeps bouncing instead of stopping. Mechanical energy is almost, but not perfectly, conserved in the guarded solver. The important takeaway about bias is that applying velocity bias during elastic repeated bounces is a bad idea because it adds energy. The current solver avoids most of that problem by not using the bias term for fast elastic impacts.
