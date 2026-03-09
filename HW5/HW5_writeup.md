# DATA SCIENCE 2VG3 Homework 5

## Files

- `rigidbody2d_friction.py`: baseline friction solver for the inclined-plane test.
- `rigidbody2d_friction_extra.py`: improved version with extra stabilization.
- `rigidbody2d_rolling.py`: rolling-body simulator and race measurements.
- `verify_hw5.py`: regenerates the reports/screenshots in `output/` and checks the measured results.

## Exercise 1

I completed `rigidbody2d_friction.py` by adding:

- rigid-body gravity updates with fixed time steps,
- collision resolution using the provided square-square contact finder,
- normal impulses with coefficient of restitution,
- tangential Coulomb friction impulses with static and kinetic limits,
- penetration correction along the contact normal,
- a headless report mode so the same scene can be measured reproducibly.

The default case is the 13 degree inclined plane with `mu_s = mu_k = 0.5` and `e = 0.1`.

## Exercise 2

The critical static-friction coefficient is

`mu_critical = tan(13 deg) = 0.230868`.

### 2(a) Why the rocking/sliding happens

The resting contact is numerically delicate because the square touches the incline through a small contact manifold, usually two corners/edges that are resolved one contact at a time. The code advances the body with a finite time step, allows a tiny penetration, and then removes that penetration with impulses and position correction. That process creates small frame-to-frame residual tangential velocities and angular velocities. With only one solver pass per frame, those residuals are not completely cancelled, so the block shows tiny rocking and creeping motion even when the friction should be strong enough to hold it.

In short, the drift comes from:

- discrete time stepping,
- sequential contact resolution for what is really a coupled two-contact resting problem,
- tiny penetration corrections that inject a small amount of tangential motion,
- friction being solved from the current frame state instead of an exact static equilibrium solve.

### 2(b) Friction-coefficient sweep

I ran the baseline engine with the same value for static and kinetic friction each time. The measured results were:

| `mu` | Final tangential speed | Average tangential acceleration over last half | Behaviour |
|---|---:|---:|---|
| 0.12 | 9.6698 | 2.1204 | Strong acceleration down the plane |
| 0.18 | 5.2557 | 1.2197 | Clear acceleration |
| 0.22 | 0.7859 | 0.1059 | Still slides because `mu < tan(theta)` |
| 0.24 | 0.0829 | 0.0032 | Above threshold, but only creeps instead of resting perfectly |
| 0.30 | 0.0526 | -0.0015 | Nearly static, small residual drift |
| 0.50 | 0.0308 | 0.0017 | Very small creep |
| 0.80 | -0.0251 | 0.0014 | Tiny oscillatory drift only |

This reproduces the expected threshold qualitatively. Below `mu = 0.230868` the square accelerates. Just above the threshold it does not accelerate strongly any more, but the baseline engine still creeps instead of sitting exactly still. So the physics trend is correct, but the resting-contact behaviour is not exact.

### 2(c) Improved strategy

My improved version is `rigidbody2d_friction_extra.py`. I used two changes:

1. More contact-solver iterations each frame (`8` instead of `1`), so the coupled contacts on the resting square are resolved more consistently.
2. A resting-contact snap: when the square has been in sustained contact, the speeds are already tiny, and `mu_s >= tan(theta)`, the code snaps it onto the exact resting configuration and zeroes the remaining drift.

This is a deliberate stabilization strategy. It does not change the low-friction cases, but it removes the artificial creeping in the supercritical cases. Measured improved results:

| `mu` | Baseline final tangential speed | Extra final tangential speed |
|---|---:|---:|
| 0.24 | 0.0829 | 0.0000 |
| 0.30 | 0.0526 | 0.0000 |
| 0.50 | 0.0308 | 0.0000 |
| 0.80 | -0.0251 | 0.0000 |

The subcritical case still behaves correctly. For example, at `mu = 0.22` the improved solver still gives a positive average tangential acceleration (`0.1059`), so it continues to slide as expected.

## Exercise 3

I completed `rigidbody2d_rolling.py` with the default scenario set to a hollow cylinder of radius `1.0` rolling down a `13 degree` incline.

Following the class approach, I extended the rigid-body code to support a `shape` property and generic shape collision handling, including true cylinder-versus-polygon contact for the rolling body on the square incline. The rolling scene reuses the friction/contact solver rather than a separate analytic kinematic update, so the measured speeds are close to theory but not exact. A small marker on the round bodies makes the rotation visible in the rendered output.

Measured hollow-cylinder speed at the end of the block:

- Analytic speed: `3.6738349528`
- Measured speed: `3.6656014919`
- Percent error: `-0.2241 %`

So the default rolling case agrees well with the expected value. The remaining small error comes from the fixed time step and discrete contact/impulse resolution.

## Exercise 4: The Race

I ran the four racers on the same `13 degree` incline and measured the speed at the end of the block.

| Racer | Measured final speed |
|---|---:|
| Sliding particle | 5.190916 |
| Sphere | 4.384223 |
| Solid cylinder | 4.234995 |
| Hollow cylinder | 3.665601 |

This matches expectations.

- The frictionless sliding particle is fastest because no energy goes into rotation.
- Among the rolling objects, the smaller the moment of inertia, the faster the translation.
- The sphere (`I = 2/5 MR^2`) is faster than the solid cylinder (`I = 1/2 MR^2`), and both are faster than the hollow cylinder (`I = MR^2`).

That ordering is what energy conservation predicts:

`v = sqrt(2gh / (1 + I/(MR^2)))`

so larger rotational inertia leaves less energy available for center-of-mass motion.

Because the rolling code now uses the actual contact solver instead of an analytic shortcut, the measured speeds are close to the theory rather than bit-for-bit identical. In my runs the race errors were all below `0.25 %`.

## Verification

I verified everything with:

```bash
python3 verify_hw5.py
```

That command regenerates the reports/screenshots in `output/` and checks:

- baseline friction: low `mu` accelerates and high `mu` nearly rests,
- extra friction: supercritical cases settle fully,
- rolling default: measured speed matches the analytic value,
- race order: `slider > sphere > solid > hollow`.

Saved artifacts:

- `output/friction_default.json`
- `output/friction_sweep.json`
- `output/friction_extra_default.json`
- `output/friction_extra_sweep.json`
- `output/rolling_default.json`
- `output/rolling_race.json`
- `output/friction_default.png`
- `output/friction_extra_default.png`
- `output/rolling_default.png`
- `output/verification_summary.json`
