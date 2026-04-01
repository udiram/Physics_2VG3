# HW7 Submission Guide

The final submission file is [rigidbody2d_cc.py](/Users/udbhavram/Documents/GitHub/Physics_2VG3/HW7/submission/rigidbody2d_cc.py).

To reproduce the results with the runner from the repo root:

```bash
python3 audit_hw7.py
```

That regenerates the audit report and screenshots in `audit_output/`.

Useful direct runs from the repo root:

```bash
python3 constraint/rigidbody2d_cc.py --headless --scenario 2 --duration 8 --pendulum-vx -2.0
python3 constraint/rigidbody2d_cc.py --headless --scenario 3 --duration 6 --bias-mode adaptive
python3 constraint/rigidbody2d_cc.py --headless --scenario 4 --duration 12 --bias-mode adaptive
```

Default submission settings:

- `scenario = 3`
- `nSphere = 3`
- `bias-mode = adaptive`
