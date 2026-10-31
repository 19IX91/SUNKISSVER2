# Unitree R1 EDU U2 Sunkiss deployment — SUNKISSVER2

This repository packages the SUNKISSVER2 controller, policy and motion reference, estimator bridge, MuJoCo replay model, validation data, and the exact x86_64 controller executable used in the latest hardware test. The full local archive is preserved separately as `SUNKISSVER2`.

**Hardware status:** The 2026-10-31 15:26 run entered Tracking for about 24.3 seconds, then hit the anchor orientation gate. The robot was down at 88.8 degrees tilt in the read-only postflight. This package is a record of that implementation; it has not demonstrated stable hardware balance.

The active motion is `deploy/robots/r1/config/policy/tracking/sunkiss/v1/params/reference_overall95.bin`, selected by `params/deploy.yaml`. Initialization and fallback are documented in `VERSION_NOTES.md`. The controller source is under `deploy/robots/r1/`.

## Dependencies

Building requires Unitree SDK2, ONNX Runtime 1.22.0, Boost program_options, yaml-cpp, fmt, Eigen, Zlib and cnpy. This repository includes cnpy source and the R1 MuJoCo meshes; the Unitree mesh license is in `third_party_licenses/`. The prebuilt executable in `bin/` depends on matching host libraries and is not a portable standalone release.

## MuJoCo replay

Run `python3 simulate_sunkiss.py --model mujoco/r1_walk_actuated.xml --output result.json` after installing the tested Python dependency versions with `python3 -m pip install -r requirements-sim.txt`. The existing results in `validation/` are simulation results and do not establish hardware safety.

## Hardware record

The latest controller and estimator logs are in `hardware_runs/2026-10-31_1526/`. Read `VERSION_NOTES.md` for the exact startup settings and outcome. The current controller behavior is described in `deploy/robots/r1/README_tracking.md`. An older tracking README is retained in `docs/` for history. This GitHub package omits copied SDK and runtime libraries.
