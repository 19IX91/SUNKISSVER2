# SUNKISSVER2 snapshot

Saved 2026-10-31 after the 15:26 hardware run. This folder preserves the current C++ controller source and executable, Python bridge and preflight scripts, exact initialization/fallback config, active policy and motion reference, runtime dependencies, validation artifacts, and latest controller/bridge logs.

## Active configuration

- Initial FSM state: FixStand.
- Motor commands are seeded from the measured pose before the FSM starts.
- FixStand blend: 2 seconds. Auto-start delay: 1 second when `R1_SUNKISS_AUTO_START=1` and `R1_SUNKISS_AUTO_START_DELAY=1`.
- The initialization/fallback YAML is identical to SUNKISSVER1. It includes the Passive fallback and LT+B transitions.
- Active reference: `deploy/robots/r1/config/policy/tracking/sunkiss/v1/params/reference_overall95.bin`, selected by `params/deploy.yaml`.
- The updated Tracking controller holds the last motor targets and gains when a tracking fault occurs. An out-of-range measured joint logs a warning and does not stop all joints. Commanded targets are limited to each joint's configured range.

## Latest hardware outcome

The 2026-10-31 15:26 run entered Tracking, continued for approximately 24.3 seconds, then logged `Anchor orientation gate`. The subsequent read-only preflight showed the robot down at 88.8 degrees tilt and estimated pelvis height 0.0669 m. No controller or bridge process remained after the run. This is a preserved version, not a successful balance validation.

The run logs are in `hardware_runs/2026-10-31_1526/`. The MuJoCo files in `validation/` are simulation-only results; they do not validate the most recent joint-range controller change on hardware.

A fresh state capture at archive time was unavailable because `enp4s0` was disconnected. `state_capture_unavailable.txt` records the DDS error. The postflight values above came from the completed read-only check immediately after the 15:26 run.
