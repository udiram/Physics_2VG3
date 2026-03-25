# 2VG3 Homework 7 Writeup

## Exercise 2: Pendulum period

For scenario 2, the pendulum length is `L = 2` and `g = 10`, so the ideal small-angle prediction is

`T = 2 pi sqrt(L / g) = 2 pi sqrt(2 / 10) = 2.8099 s`.

I measured the period from successive zero crossings of the bob's `x` coordinate. The crossing times are linearly interpolated between frames, and the period estimate is twice the time between consecutive crossings.

Measured results:

- `v_x = -0.5`: `T = 2.8331 s` (error `0.0232 s`, about `0.83%`)
- `v_x = -1.0`: `T = 2.8387 s` (error `0.0288 s`, about `1.02%`)
- `v_x = -2.0`: `T = 2.8612 s` (error `0.0513 s`, about `1.83%`)
- `v_x = -4.0`: `T = 2.9534 s` (error `0.1435 s`, about `5.11%`)

Conclusion:

- For small initial speeds, the measured period stays close to the ideal value.
- The measured period increases as the initial speed increases.
- This confirms the expected small-angle behavior: the textbook formula is a better approximation when the motion starts with a small speed, roughly below `1` here.

## Exercise 3: Newton's cradle with Catto bias

I used `nSphere = 3` as the default and started the leftmost sphere with `v = (-2, 0, 0)`.

I also introduced a very small initial gap of `0.001` between neighboring spheres. Without that gap, the discrete solver treated the resting balls as one persistent contact chain and spread the motion across the middle spheres too early.

With the standard Catto collision bias,

`vbias = 0 if (dclose < 0.01) else dclose * 0.3 / dt`

the cradle runs, but the first transfer is still too energetic. At the first far-end apex (`t = 2.1667 s`):

- the rightmost sphere moves to `x = 1.5133`, which is a displacement of `1.0123` from its rest position,
- the left and middle spheres are still displaced by about `0.2413` and `0.1437`,
- so the transfer is visibly less clean than an ideal one-sphere handoff.

## Exercise 4: Zero collision bias

With `vbias = 0`, the cradle behavior is much cleaner. At the same first far-end apex (`t = 2.1667 s`):

- the rightmost sphere moves to `x = 1.2457`, a displacement of `0.7447` from rest,
- the left and middle spheres stay much closer to rest, with residual displacements of about `0.0539` and `0.0425`.

This is much closer to the expected Newton's cradle motion, where only the far-end sphere should have a large swing after impact.

## Exercise 5: Alternative bias idea

I used an adaptive collision bias:

`vbias = base * max(0, 1 - min(abs(unormal) / v_threshold, 1))`

with

- `base = dclose * 0.3 / dt`
- `v_threshold = 0.75`

Reasoning:

- fast impacts such as Newton's cradle should not receive much positional bias, because that can inject extra energy,
- slow or resting contacts should still keep a penetration-correction term active.

Observed behavior:

- For the `nSphere = 3` cradle, the adaptive bias gives the same first-transfer result as zero bias in this test. That is expected, because the impact speed is large enough that the speed scaling drives the bias almost to zero during the collision.
- For `nSphere = 5`, the setup still works. At the first far-end apex (`t = 2.2000 s`), the rightmost sphere is displaced by `0.6525` from rest while the intermediate spheres stay within about `0.055` of their rest positions.
- For scenario 4, I tested the four-block stack for `12 s` with Catto, zero, and adaptive bias. All three stayed stable. The final block gaps were about `0.6957`, `0.6971`, and `0.6982`, and the maximum vertical speed after the first `2 s` was about `0.0081`.

Overall conclusion:

- Standard Catto bias is too aggressive for the cradle test.
- Zero bias gives a much cleaner one-sphere transfer.
- The adaptive bias is the best general choice here: it behaves like zero bias for fast cradle impacts, but still keeps the standard penetration-correction structure for slower contacts and stacking.

## Deliverable note

The code file is `constraint/rigidbody2d_cc.py`, with scenario `3` as the default. The Newton's cradle setup uses `nSphere = 3` by default and a cradle gap of `0.001`.
