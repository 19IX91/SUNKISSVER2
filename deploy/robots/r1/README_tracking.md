# Sunkiss Tracking controller in SUNKISSVER2

This directory contains the R1 EDU U2 Sunkiss C++ controller source. The exact x86_64 executable copied from the last local build is at `../../../bin/r1_ctrl_autostart_linux_x86_64`; it is not a standalone portable binary. See the repository root `README.md` and `VERSION_NOTES.md` for status and dependencies.

## Current startup

`main.cpp` seeds the initial low-level joint targets from measured motor positions. `config/sunkiss.yaml` starts in FixStand and blends to the first motion reference over two seconds. With `R1_SUNKISS_AUTO_START=1`, the controller waits for the estimator bridge and entry checks, then starts Tracking after a one-second countdown when `R1_SUNKISS_AUTO_START_DELAY=1`.

The active motion is selected in `config/policy/tracking/sunkiss/v1/params/deploy.yaml` and is `reference_overall95.bin` in this version.

## Tracking behavior

The controller reads the trained 50 Hz observations and policy actions. One measured joint outside the configured range now produces a warning instead of freezing all joints. Each motor target is individually limited to its configured joint range. Invalid/nonfinite state, stale pelvis odometry, policy timing faults, and excessive anchor height/orientation error still stop policy updates. At a Tracking fault or completion, the controller holds the last commanded motor targets and gains.

The saved `config/sunkiss.yaml` retains SUNKISSVER1 initialization and its Passive fallback on LowState timeout. This fallback is separate from the Tracking fault handler. Neither holding a target nor entering Passive is a balance recovery controller.

## Validation status

The 2026-10-31 15:26 hardware run entered Tracking for about 24.3 seconds, faulted on the anchor orientation gate, and ended with the robot down at 88.8 degrees tilt. The MuJoCo replay files are approximate simulation checks and do not establish hardware balance. The repository preserves this implementation for review and further work; it is not a successful deployment release.

`docs/HISTORICAL_README_tracking.md` is retained as an earlier document and does not describe this exact version.
