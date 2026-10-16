# Genesis Validation

This document records local validation results for the OneRobotics A1 robot models using Genesis World.

The results were obtained with Genesis World 1.4.4 on the CPU backend using float32 precision.

Standalone Python package installation and distribution validation should be recorded separately.

## Environment (reported October 15, 2026)

- Genesis World 1.4.4
- CPU backend, float32 precision
- Simulation with original OneRobotics A1 URDF/STL files

## Reported results

| Configuration | URDF loading | Joint control | IK |
| --- | --- | --- | --- |
| A1 right arm (7 DoF) | Passed | Executed | Passed |
| A1 left arm (7 DoF) | Passed | Executed | Passed |
| A1 bimanual stand (14 DoF) | Passed | Executed | Passed |

Recorded right-arm validation results:

- Seven-joint position control endpoint RMSE: approximately **0.0144 rad**; J2 offset approximately **0.038 rad**.
- Joint velocity sample: **0.0991 rad/s** against **0.1 rad/s** target.
- A **0.5 Nm** force command was issued; no independent torque-fidelity measurement was performed.
- IK visual demo: **300 reachable FK-derived targets**, maximum observed end-link position error **0.000040 m**.

No independent physical fidelity, calibrated contact response, hardware transfer, or production controller guarantee is implied.

## Rechecking the standalone package

From the project root, after installing the package:

```bash
python -c 'from onerobotics_a1_genesis import urdf_path; print(urdf_path("right")); print(urdf_path("left")); print(urdf_path("bimanual"))'
a1-genesis-demo --robot right --check-assets
a1-genesis-demo --robot left --check-assets
a1-genesis-demo --robot bimanual --check-assets
a1-genesis-demo --robot right --demo view -v
a1-genesis-demo --robot left --demo view -v
a1-genesis-demo --robot bimanual --demo view -v
```

Use `--demo control` and `--demo ik` to check each claimed feature before a release. Record the actual Genesis/Python versions and measured results for the standalone installation.

## Known source-model limitations

- Independent-arm Link7 temporary flange inertial values are preserved.
- Bimanual URDF effort/velocity limits contain source zero placeholders rather than actuator ratings.
- Runtime force limit overrides use peak-torque metadata; they do not change the source URDF or establish hardware fidelity.
- No grippers are provided.
