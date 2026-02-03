# Tests

This folder contains minimal unit tests for the project.

## What was added

- [tests/test_robot_model.py](test_robot_model.py):
  - Loads `UnitreeA1` from the package.
  - Verifies key parameters are positive.
  - Checks kinematics (`get_foot_position`, `is_kinematically_feasible`).
  - Checks torque calculation (`compute_joint_torques_from_force`).

- [tests/conftest.py](conftest.py):
  - Adds `src/` to `sys.path` so tests can import the package without installing it.

## How to run

From the project root, run:

```
C:/Users/Acer/OneDrive/Desktop/unitTreeA1_Quadruped_Robot/.venv/Scripts/python.exe -m pytest
```

Expected output includes:

```
2 passed
```

## Notes

- Results are printed in the terminal by default.
- If you install the package in editable mode, you can remove the `sys.path` helper in `conftest.py`.
