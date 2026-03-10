# 2VG3 HW5 Written Answers (Draft)

## Exercise 2a
The slight rocking/sliding is a numerical artifact of the contact solver. We treat each contact independently and then average, rather than solving all contacts simultaneously, so the normal/tangential impulses do not cancel perfectly. With gravity on, each frame injects a small downward velocity, and the discrete integration plus position correction introduces tiny residual errors. Those errors show up as jitter and a slow drift even when the theoretical static-friction condition is satisfied.

## Exercise 2b
For an incline of 13? (0.227 rad), the theoretical threshold for no acceleration is ?_s = tan(13?) ? 0.231. When ? is well above this value, the box should be stationary; in the simulation it usually still jitters and may drift at nearly constant speed because of the numerical issues in 2a. As ? approaches ~0.23, the drift becomes more noticeable. Once ? drops below ~0.23, the box should transition to clear acceleration down the plane rather than just a slow constant-speed slide.

## Exercise 2c (Extra version)
I improved stability by adding a simple “sleep”/resting state and keeping full tangential position correction only when the contact is inside the static friction cone. Concretely, I track small velocities over several frames and, once both linear and angular speeds stay below a small threshold for a few frames, I freeze the body by zeroing `vel`/`omega` and disabling gravity for that body. This removes the residual jitter caused by gravity nudging the contact every frame. When the contact is sliding (outside the static cone), I only apply a scaled tangential position correction so the block can still move; when it’s static, I fully undo tangential drift. This combination reduces rocking/sliding at the critical angle without breaking normal sliding behavior.

## Exercise 4 (Race)
Expected ordering from fastest to slowest (for the same incline, neglecting losses):
1. Sliding particle (no rotation) fastest.
2. Sphere (I = 2/5 MR^2)
3. Solid cylinder (I = 1/2 MR^2)
4. Hollow cylinder (I = MR^2) slowest.

Expected acceleration formula:
    a = g sin(theta) / (1 + I/(MR^2))

Measurements (fill in after running):
- Hollow cylinder final speed: ______
- Solid cylinder final speed: ______
- Sphere final speed: ______
- Sliding particle final speed: ______

Comments (fill in):
- If the measured ordering differs, possible causes include energy loss from polygonal collision, friction model approximations, or numerical integration errors.
