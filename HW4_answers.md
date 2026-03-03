# 2VG3 Homework 4 - Collision Results

Generated from `rigidbody2d_collision.py`.

## Coefficient of Restitution e = 1

### Scenario 0
Default: no initial rotation.
Collision events:
- Event 1: t=2.233000 s, normal=(1.000000, 0.000000, -0.000000), avg_contact=(0.266500, 25.000000, 0.000000), type=averaged.
-   Raw contacts: (0.266000, 25.000000, -1.300000), (0.267000, 25.000000, 1.300000)
-   Corner body: undetermined; Face body: undetermined. Note: contact geometry is symmetric.
- Event 2: t=2.492000 s, normal=(0.661757, 0.000000, 0.749718), avg_contact=(0.007500, 25.000000, 0.000000), type=averaged.
-   Raw contacts: (0.394639, 25.000000, -0.342105), (-0.379639, 25.000000, 0.342105)
-   Corner body: both; Face body: both. Note: face-face simultaneous contacts.
First-contact conservation numbers:
- Before: P=(-2.00000000, 0.00000000), L=3.00000000, K_trans=10.00000000, K_rot=0.00000000, K_total=10.00000000
- After:  P=(-2.00000000, 0.00000000), L=3.00000000, K_trans=2.85950413, K_rot=7.14049587, K_total=10.00000000
Energy discussion:
- With e=1, total kinetic energy remains nearly unchanged; translational and rotational parts trade with each other via off-centre impulse.

### Scenario 1
Body 0 starts at 30 degrees.
Collision events:
- Event 1: t=2.123500 s, normal=(1.000000, 0.000000, -0.000000), avg_contact=(0.704000, 25.000000, -1.157203), type=averaged.
-   Raw contacts: (0.704000, 25.000000, -1.160489), (0.704000, 25.000000, -1.153917)
-   Corner body: both; Face body: both. Note: face-face simultaneous contacts.
- Event 2: t=2.257500 s, normal=(0.958427, 0.000000, 0.285339), avg_contact=(-0.288900, 25.000000, 0.991331), type=averaged.
-   Raw contacts: (-0.288320, 25.000000, 0.989382), (-0.289480, 25.000000, 0.993279)
-   Corner body: both; Face body: both. Note: face-face simultaneous contacts.
First-contact conservation numbers:
- Before: P=(-2.00000000, 0.00000000), L=3.00000000, K_trans=10.00000000, K_rot=0.00000000, K_total=10.00000000
- After:  P=(-2.00000000, 0.00000000), L=3.00000000, K_trans=2.50499225, K_rot=7.49500775, K_total=10.00000000
Energy discussion:
- With e=1, total kinetic energy remains nearly unchanged; translational and rotational parts trade with each other via off-centre impulse.

### Scenario 2
Body 0 starts at 60 degrees; body 1 starts at 30 degrees.
Collision events:
- Event 1: t=2.109500 s, normal=(0.866025, 0.000000, 0.500000), avg_contact=(0.678505, 25.000000, 0.158846), type=averaged.
-   Raw contacts: (0.678675, 25.000000, 0.158550), (0.678334, 25.000000, 0.159141)
-   Corner body: both; Face body: both. Note: face-face simultaneous contacts.
- Event 2: t=2.433000 s, normal=(0.999453, 0.000000, 0.033082), avg_contact=(-0.262000, 25.000000, 0.225449), type=averaged.
-   Raw contacts: (-0.261881, 25.000000, 0.221860), (-0.262119, 25.000000, 0.229038)
-   Corner body: both; Face body: both. Note: face-face simultaneous contacts.
First-contact conservation numbers:
- Before: P=(-2.00000000, 0.00000000), L=3.00000000, K_trans=10.00000000, K_rot=0.00000000, K_total=10.00000000
- After:  P=(-2.00000000, 0.00000000), L=3.00029321, K_trans=3.58035195, K_rot=6.41964805, K_total=10.00000000
Energy discussion:
- With e=1, total kinetic energy remains nearly unchanged; translational and rotational parts trade with each other via off-centre impulse.

### Scenario 3
Body 0 starts with omega=-5 rad/s; body 1 is moved to (-1.9, 25, 1.9) and starts at rest.
Collision events:
- Event 1: t=0.051000 s, normal=(1.000000, 0.000000, -0.000000), avg_contact=(-3.700000, 25.000000, 0.778640), type=averaged.
-   Raw contacts: (-3.700000, 25.000000, 0.774481), (-3.700000, 25.000000, 0.782798)
-   Corner body: both; Face body: both. Note: face-face simultaneous contacts.
First-contact conservation numbers:
- Before: P=(2.00000000, 0.00000000), L=-2.33333333, K_trans=2.00000000, K_rot=8.33333333, K_total=10.33333333
- After:  P=(2.00000000, 0.00000000), L=-2.33333333, K_trans=3.71689629, K_rot=6.61643704, K_total=10.33333333
Energy discussion:
- With e=1, total kinetic energy remains nearly unchanged; translational and rotational parts trade with each other via off-centre impulse.

### Scenario 4
Body 0 starts with omega=+3 rad/s; body 1 starts at -10 degrees.
Collision events:
- Event 1: t=2.163000 s, normal=(0.978587, 0.000000, 0.205833), avg_contact=(0.262558, 25.000000, -0.957227), type=averaged.
-   Raw contacts: (0.263251, 25.000000, -0.960523), (0.261865, 25.000000, -0.953931)
-   Corner body: both; Face body: both. Note: face-face simultaneous contacts.
- Event 2: t=2.284000 s, normal=(0.881251, 0.000000, 0.472649), avg_contact=(-0.077460, 25.000000, 1.215218), type=averaged.
-   Raw contacts: (-0.076809, 25.000000, 1.214005), (-0.078111, 25.000000, 1.216431)
-   Corner body: both; Face body: both. Note: face-face simultaneous contacts.
First-contact conservation numbers:
- Before: P=(-2.00000000, 0.00000000), L=5.00000000, K_trans=10.00000000, K_rot=3.00000000, K_total=13.00000000
- After:  P=(-2.00000000, 0.00000000), L=5.00137667, K_trans=1.94170055, K_rot=11.05829945, K_total=13.00000000
Energy discussion:
- With e=1, total kinetic energy remains nearly unchanged; translational and rotational parts trade with each other via off-centre impulse.

## Coefficient of Restitution e = 0

### Scenario 0
Default: no initial rotation.
Collision events:
- Event 1: t=2.233000 s, normal=(1.000000, 0.000000, -0.000000), avg_contact=(0.266500, 25.000000, 0.000000), type=averaged.
-   Raw contacts: (0.266000, 25.000000, -1.300000), (0.267000, 25.000000, 1.300000)
-   Corner body: undetermined; Face body: undetermined. Note: contact geometry is symmetric.
First-contact conservation numbers:
- Before: P=(-2.00000000, 0.00000000), L=3.00000000, K_trans=10.00000000, K_rot=0.00000000, K_total=10.00000000
- After:  P=(-2.00000000, 0.00000000), L=3.00000000, K_trans=1.66942149, K_rot=1.78512397, K_total=3.45454545
Energy discussion:
- With e=0, relative normal motion is damped at impact, so translational kinetic energy drops and some energy appears as rotation; total kinetic energy decreases.

### Scenario 1
Body 0 starts at 30 degrees.
Collision events:
- Event 1: t=2.123500 s, normal=(1.000000, 0.000000, -0.000000), avg_contact=(0.704000, 25.000000, -1.157203), type=averaged.
-   Raw contacts: (0.704000, 25.000000, -1.160489), (0.704000, 25.000000, -1.153917)
-   Corner body: both; Face body: both. Note: face-face simultaneous contacts.
- Event 2: t=2.300500 s, normal=(0.932726, 0.000000, 0.360587), avg_contact=(-0.369177, 25.000000, 1.421141), type=averaged.
-   Raw contacts: (-0.345916, 25.000000, 1.360972), (-0.392438, 25.000000, 1.481310)
-   Corner body: both; Face body: both. Note: face-face simultaneous contacts.
- Event 3: t=2.316500 s, normal=(0.926541, 0.000000, 0.376193), avg_contact=(0.461166, 25.000000, -0.685031), type=averaged.
-   Raw contacts: (0.789964, 25.000000, -1.494842), (0.132368, 25.000000, 0.124780)
-   Corner body: both; Face body: both. Note: face-face simultaneous contacts.
- Event 4: t=2.320000 s, normal=(0.926187, 0.000000, 0.377065), avg_contact=(-0.288505, 25.000000, 1.148798), type=averaged.
-   Raw contacts: (-0.147125, 25.000000, 0.801526), (-0.429885, 25.000000, 1.496071)
-   Corner body: both; Face body: both. Note: face-face simultaneous contacts.
- Event 5: t=2.333500 s, normal=(0.925331, 0.000000, 0.379161), avg_contact=(0.702059, 25.000000, -1.308129), type=averaged.
-   Raw contacts: (0.781316, 25.000000, -1.501554), (0.622802, 25.000000, -1.114703)
-   Corner body: both; Face body: both. Note: face-face simultaneous contacts.
First-contact conservation numbers:
- Before: P=(-2.00000000, 0.00000000), L=3.00000000, K_trans=10.00000000, K_rot=0.00000000, K_total=10.00000000
- After:  P=(-2.00000000, 0.00000000), L=3.00000000, K_trans=5.46641995, K_rot=1.87375194, K_total=7.34017188
Energy discussion:
- With e=0, relative normal motion is damped at impact, so translational kinetic energy drops and some energy appears as rotation; total kinetic energy decreases.

### Scenario 2
Body 0 starts at 60 degrees; body 1 starts at 30 degrees.
Collision events:
- Event 1: t=2.109500 s, normal=(0.866025, 0.000000, 0.500000), avg_contact=(0.678505, 25.000000, 0.158846), type=averaged.
-   Raw contacts: (0.678675, 25.000000, 0.158550), (0.678334, 25.000000, 0.159141)
-   Corner body: both; Face body: both. Note: face-face simultaneous contacts.
- Event 2: t=2.520000 s, normal=(0.999888, 0.000000, 0.014964), avg_contact=(-0.062302, 25.000000, -0.744033), type=averaged.
-   Raw contacts: (-0.062276, 25.000000, -0.745769), (-0.062328, 25.000000, -0.742296)
-   Corner body: both; Face body: both. Note: face-face simultaneous contacts.
First-contact conservation numbers:
- Before: P=(-2.00000000, 0.00000000), L=3.00000000, K_trans=10.00000000, K_rot=0.00000000, K_total=10.00000000
- After:  P=(-2.00000000, 0.00000000), L=3.00029321, K_trans=4.27344958, K_rot=1.60491201, K_total=5.87836159
Energy discussion:
- With e=0, relative normal motion is damped at impact, so translational kinetic energy drops and some energy appears as rotation; total kinetic energy decreases.

### Scenario 3
Body 0 starts with omega=-5 rad/s; body 1 is moved to (-1.9, 25, 1.9) and starts at rest.
Collision events:
- Event 1: t=0.051000 s, normal=(1.000000, 0.000000, -0.000000), avg_contact=(-3.700000, 25.000000, 0.778640), type=averaged.
-   Raw contacts: (-3.700000, 25.000000, 0.774481), (-3.700000, 25.000000, 0.782798)
-   Corner body: both; Face body: both. Note: face-face simultaneous contacts.
First-contact conservation numbers:
- Before: P=(2.00000000, 0.00000000), L=-2.33333333, K_trans=2.00000000, K_rot=8.33333333, K_total=10.33333333
- After:  P=(2.00000000, 0.00000000), L=-2.33333333, K_trans=1.10507356, K_rot=3.67133132, K_total=4.77640488
Energy discussion:
- With e=0, relative normal motion is damped at impact, so translational kinetic energy drops and some energy appears as rotation; total kinetic energy decreases.

### Scenario 4
Body 0 starts with omega=+3 rad/s; body 1 starts at -10 degrees.
Collision events:
- Event 1: t=2.163000 s, normal=(0.978587, 0.000000, 0.205833), avg_contact=(0.262558, 25.000000, -0.957227), type=averaged.
-   Raw contacts: (0.263251, 25.000000, -0.960523), (0.261865, 25.000000, -0.953931)
-   Corner body: both; Face body: both. Note: face-face simultaneous contacts.
- Event 2: t=2.373500 s, normal=(0.940459, 0.000000, 0.339907), avg_contact=(-0.477915, 25.000000, 1.520816), type=averaged.
-   Raw contacts: (-0.476695, 25.000000, 1.517442), (-0.479134, 25.000000, 1.524190)
-   Corner body: both; Face body: both. Note: face-face simultaneous contacts.
First-contact conservation numbers:
- Before: P=(-2.00000000, 0.00000000), L=5.00000000, K_trans=10.00000000, K_rot=3.00000000, K_total=13.00000000
- After:  P=(-2.00000000, 0.00000000), L=5.00137667, K_trans=2.57722752, K_rot=2.67729775, K_total=5.25452527
Energy discussion:
- With e=0, relative normal motion is damped at impact, so translational kinetic energy drops and some energy appears as rotation; total kinetic energy decreases.
