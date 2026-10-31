# R1 Sunkiss motion-tracking deployment

> **Read first:** [`IMPORTANT_R1_SUNKISS_PROCEDURE.md`](../../../IMPORTANT_R1_SUNKISS_PROCEDURE.md) records the current staged procedure and its validation limits.

A dedicated tracking state and deployment package are now provided for the best
validated Sunkiss model. The original walking configuration is preserved.
No physical robot commands were sent during preparation.

## Required files

Keep these paths together under `deploy/robots/r1/`:

- `config/sunkiss.yaml`: separate controller/FSM configuration.
- `config/policy/tracking/sunkiss/v1/params/deploy.yaml`: runtime parameters.
- `config/policy/tracking/sunkiss/v1/params/reference_footwork_reduced.bin`: adjusted 1,326-frame reference used by the current Sunkiss config.
- `config/policy/tracking/sunkiss/v1/params/reference.bin`: untouched source reference.
- `config/policy/tracking/sunkiss/v1/exported/policy.onnx`: selected model.
- `config/policy/tracking/sunkiss/v1/package_manifest.json`: source/file hashes.
- New tracking headers and `src/State_Tracking.cpp`: controller implementation.

Build on the robot's CPU architecture; a workstation x86-64 executable is not an
ARM executable. CMake selects the bundled x86-64 or aarch64 ONNX Runtime.
Install the usual Unitree SDK2, DDS, Eigen, Boost, yaml-cpp, and fmt dependencies.

```bash
cmake -S deploy/robots/r1 -B deploy/robots/r1/build \
  -DCMAKE_BUILD_TYPE=Release -DUNITREE_SDK2_ROOT=/path/to/unitree_sdk2
cmake --build deploy/robots/r1/build -j4
ctest --test-dir deploy/robots/r1/build --output-on-failure
```

`UNITREE_SDK2_ROOT` is optional for a system-installed SDK. When Conda supplies
conflicting yaml-cpp/fmt headers, select system YAML using `-Dyaml-cpp_DIR=...`.

## State-estimation input

The policy requires pelvis position/orientation and body-frame linear velocity
in addition to joint encoders and IMU gyro. The new controller consumes an estimate;
it does not substitute zeros or provide an unvalidated IMU integration fallback.

The [official R1 SDK joint map](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/dds_wrapper/robots/r1/defines.h)
confirms body slots 0–13, 15–19, 22–26, and head slots **29/30**.
The public [R1 locomotion client](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/r1/loco/r1_loco_client.hpp)
does not establish that usable pelvis odometry remains available in low-level mode
on this particular robot. Verify an actual feed on its firmware.

On the robot, with its ROS 2 environment sourced, discover odometry without
publishing motor commands:

```bash
python3 scripts/r1_odometry_bridge.py --list-topics
ros2 topic list -t
```

The bridge supports `nav_msgs/msg/Odometry`. Pose must describe the **MuJoCo pelvis
origin** in a right-handed z-up world with the floor at **z=0**. Twist must be
linear velocity in pelvis/body axes. Camera/torso poses and world-frame velocity
must first be transformed by the upstream estimator. Verify the feed remains
accurate and live after the factory walking controller releases control.

Use the actual verified topic and frame names, on the same computer as the controller:

```bash
python3 scripts/r1_odometry_bridge.py --topic /verified_pelvis_odometry \
  --world-frame odom --base-frame pelvis
```

That topic name is an example, not a verified R1 topic. If no appropriate feed
exists, a physical state estimator must be configured before enabling tracking.
ROS 2/rclpy/nav_msgs are required for this bridge, not for the C++ controller.
Another estimator may send the same localhost UDP protocol on port 15150: JSON
`version: 1`, `frame: pelvis`, `stamp_ns` (host monotonic measurement time),
`position_w` (3), `quaternion_wxyz` (4), `linear_velocity_b` (3).
The bridge preserves source age; retransmission does not refresh old measurements.

## Controller behavior

After physical low-level-control prerequisites and the estimator are verified,
select the separate configuration explicitly:

```bash
./deploy/robots/r1/build/r1_ctrl --config sunkiss.yaml --network YOUR_ROBOT_INTERFACE
```

This command sends real motor commands and was **not executed** during preparation.
The Sunkiss config starts directly in FixStand. Before starting it, put the
robot in its standing/FixStand posture. After releasing the high-level motion
owner, the executable seeds LowCmd from the measured joint pose, then applies
active FixStand gains and blends to the first reference over two seconds. This
avoids a zero-position-gain interval after the handoff. `RT + A` requests Tracking
in the manual-start build. The Sunkiss FSM has no Passive state or transition.
Tracking faults, completion, or low-state timeouts route to ActiveHold, which
holds the measured joint pose with active gains; this is not balance recovery.

To skip the `RT + A` press, use the Sunkiss-only auto-start build. After the
two-second blend, it waits for fresh pelvis odometry and configured Tracking
entry checks, then counts down for one second by default before requesting Tracking.
If the bridge or entry pose is not ready, it stays in the active FixStand state
and reports why. Set
`R1_SUNKISS_AUTO_START_DELAY` to an integer from 0 to 30 seconds to add a
countdown. The stock walking FixStand state is unchanged.

```bash
# Start this first; it supplies the localhost UDP pelvis feed required by Tracking.
python3 scripts/r1_estimator_udp_bridge.py --interface enp4s0
# Confirm Sunkiss config and model load without initializing DDS.
./deploy/robots/r1/build/r1_ctrl_autostart --config sunkiss.yaml --check-config
# After the robot is upright and supported, start the controller.
R1_SUNKISS_AUTO_START=1 R1_SUNKISS_AUTO_START_DELAY=1 \
  ./deploy/robots/r1/build/r1_ctrl_autostart \
  --config sunkiss.yaml --network YOUR_ROBOT_INTERFACE
```

The historical C++ fault path did transition to Passive after an orientation
guard failure. The current code removes that Sunkiss transition and uses
ActiveHold. It has not been exercised on hardware. Keep the estimator bridge
running throughout; do not launch it while the robot is down.

The 26.50-second sequence ends in ActiveHold. ActiveHold preserves joint pose
with position gains but does not provide active balance recovery.
Physical end-effector error is not reconstructed here, so these runtime guards
do not reproduce every simulation termination test.

The engine uses the trained 50 Hz observations, joint order, offsets, gains, and
per-joint action scales. Startup aligns reference yaw/XY while retaining calibrated
floor height. Head motors hold entry positions; reserved slots are disabled.
`effort_limits` records simulation assumptions; it does not reprogram firmware
torque/current limits. Physical motor limits and IMU/pelvis alignment still need
verification on the actual robot.

## Verification completed locally

- SDK-linked `r1_ctrl` and SDK-independent `r1_tracking_offline` build successfully.
- CTest checks missing, stale, duplicate, future, wrong-frame, malformed and
  nonfinite estimates; Python tests check the ROS bridge contract.
- The actual C++ ONNX engine completes all 1,325 strict MuJoCo steps (26.50 s),
  matching observations within 7.2e-7 and actions within 8.6e-6 versus Python.
  Report: `logs/r1_deployment_validation_v2/validation.json`.
- A translated/rotated-world replay checks startup alignment for another complete
  26.50 s, with differences below 9.6e-7 / 5.8e-6.
  Report: `logs/r1_deployment_alignment_validation/validation.json`.
- Starting from rest at the first reference pose, combined with yaw/XY alignment,
  also completes the full replay. Differences stay below 8.4e-7 / 7.7e-6.
  Report: `logs/r1_deployment_standing_start_validation_v2/validation.json`.
- Simulation ground truth emulates the estimator in these tests. Physical
  estimation, live DDS wiring, motor firmware behavior, and hardware execution
  have not been validated.

Repeat the SDK-independent replay with:

```bash
MUJOCO_GL=glfw python scripts/validate_r1_deployment.py \
  --package logs/rsl_rl/r1_tracking/sunkiss_20261028/best_model \
  --deploy-yaml deploy/robots/r1/config/policy/tracking/sunkiss/v1/params/deploy.yaml \
  --output-dir logs/r1_deployment_validation_new
```

Regenerate another package using `scripts/prepare_r1_tracking_deploy.py --package ...
--output-dir NEW_DIRECTORY`. Existing deployment packages are preserved.
