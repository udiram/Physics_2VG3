# 2VG3 HW2 Written Answers

## Exercise 1
Using the provided `spring.py` integrator (semi-implicit Euler) with its default clamp `dt <= 0.02`, the largest spring constant before numerical blow-up is approximately:

- `k_stable ≈ 5.0 x 10^3`

This matches the linear stability bound for two equal masses:

- `omega = sqrt(2k)`, and stability requires `dt * omega < 2`
- so `k < 2 / dt^2 = 2 / (0.02)^2 = 5000`

## Exercise 3
For two equal masses (`m=1`) with spring damping force `D (v_rel · n) n`, the extension dynamics are:

- `e'' + 2 D e' + 2 k e = 0`

Critical damping occurs when `D = sqrt(2k)`.  
With `k=1`:

- `D_critical = sqrt(2) ≈ 1.4142`

## Exercise 4
Required initial speed for each mass to get steady separation `r=20` with `L=15`, `k=10`, `m=1`:

- Spring force magnitude: `F_s = k(r-L) = 10(20-15)=50`
- Each mass orbits COM at radius `r/2 = 10`, so centripetal condition:
  - `m v^2 / (r/2) = F_s`
  - `v^2 = F_s (r/2) / m = 50*10 = 500`
  - `v = sqrt(500) ≈ 22.3607`

So each mass should start with speed:

- `|v_0| ≈ 22.3607`

in opposite tangential directions so total momentum is zero and COM stays fixed.

## Why can steady separation be larger than rest length?
The spring rest length `L` is the zero-force length, not a fixed geometric constraint.  
In rotating motion, spring tension must provide centripetal force:

- `k(r-L) = 2 m v^2 / r`

For any nonzero angular speed, the right-hand side is positive, so `r-L > 0`, hence:

- `r > L`

Therefore, a rotating two-mass spring system has a stretched equilibrium radius set by speed/angular momentum.
