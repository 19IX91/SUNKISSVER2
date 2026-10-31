#!/usr/bin/env python3
"""Read-only R1 Sunkiss hardware preflight; never publishes rt/lowcmd."""
from __future__ import annotations

import argparse
import json
import math
import threading
import time
from pathlib import Path

import numpy as np

from sunkiss_core import MOTOR_IDS, SunkissCore, State, quat_matrix


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--seconds", type=float, default=5.)
    parser.add_argument("--timeout", type=float, default=8.)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not (0 < args.seconds <= 30 and 0 < args.timeout <= 60):
        parser.error("seconds must be <=30 and timeout <=60")

    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
        from unitree_sdk2py.utils.crc import CRC
        from estimator_data_idl import EstimatorData_
    except ImportError as e:
        raise SystemExit(f"Unitree SDK/EstimatorData IDL unavailable: {e}")

    latest = {"low": None, "est": None, "low_at": 0., "est_at": 0.}
    lock = threading.Lock()
    def on_low(m):
        with lock:
            latest.update(low=m, low_at=time.monotonic())
    def on_est(m):
        with lock:
            latest.update(est=m, est_at=time.monotonic())

    try:
        ChannelFactoryInitialize(0, args.interface)
    except Exception as e:
        raise SystemExit(f"DDS initialization failed on {args.interface}: {e}")
    switcher = MotionSwitcherClient(); switcher.SetTimeout(2.); switcher.Init()
    owner_code, owner = switcher.CheckMode()
    lowcmd_samples = [0]
    lowcmd_sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
    lowcmd_sub.Init(lambda _m: lowcmd_samples.__setitem__(0, lowcmd_samples[0]+1), 1)
    low_sub = ChannelSubscriber("rt/lowstate", LowState_)
    est_sub = ChannelSubscriber("rt/EstimatorData", EstimatorData_)
    low_sub.Init(on_low, 10); est_sub.Init(on_est, 10)
    crc = CRC(); core = SunkissCore(Path(__file__).resolve().parents[1] / "config/policy/tracking/sunkiss/v1/params/deploy.yaml")
    deadline = time.monotonic() + args.timeout
    last_tick = None; unique = 0; bad_crc = 0; max_abs_action = 0.; last_report = None
    try:
        while time.monotonic() < deadline:
            with lock:
                low, est = latest["low"], latest["est"]
                low_at, est_at = latest["low_at"], latest["est_at"]
            if low is not None and est is not None:
                if time.monotonic()-low_at > .1 or time.monotonic()-est_at > .1:
                    time.sleep(.01); continue
                if int(low.tick) != last_tick:
                    last_tick = int(low.tick); unique += 1
                    if crc.Crc(low) != low.crc:
                        bad_crc += 1; time.sleep(.01); continue
                    q = np.asarray([low.motor_state[i].q for i in MOTOR_IDS], np.float32)
                    dq = np.asarray([low.motor_state[i].dq for i in MOTOR_IDS], np.float32)
                    imu = low.imu_state
                    s = State(np.asarray(est.position_2d, np.float32),
                              np.asarray(imu.quaternion, np.float32),
                              np.asarray(est.v_body_3d, np.float32),
                              np.asarray(imu.gyroscope, np.float32), q, dq).checked()
                    core.align(s)
                    obs = core.observation(0, s, np.zeros(24, np.float32))
                    action = core.infer(obs)
                    max_abs_action = max(max_abs_action, float(np.max(np.abs(action))))
                    joint_error, height_error = core.entry_error(s)
                    joint_deltas = np.abs(s.q-core.reference[0].q)
                    names = core.config["joint_names"]
                    R = quat_matrix(s.orientation)
                    tilt = math.degrees(math.acos(float(np.clip(R[2, 2], -1, 1))))
                    last_report = {
                        "mode_machine": int(low.mode_machine), "mode_pr": int(low.mode_pr),
                        "motor_count": len(low.motor_state), "motor_error_count": sum(bool(low.motor_state[i].motorstate) for i in MOTOR_IDS),
                        "estimator_position_2d": s.position.tolist(), "estimator_body_velocity": s.velocity_body.tolist(),
                        "imu_quaternion_wxyz": s.orientation.tolist(), "imu_tilt_deg": tilt,
                        "max_joint_speed_rad_s": float(np.max(np.abs(s.dq))),
                        "first_reference_joint_error_rad": joint_error,
                        "largest_start_pose_joint_errors_rad": sorted(
                            ({"joint": names[i], "error_rad": float(joint_deltas[i]),
                              "measured_rad": float(s.q[i]), "reference_rad": float(core.reference[0].q[i])}
                             for i in range(24)), key=lambda x: x["error_rad"], reverse=True)[:8],
                        "first_reference_height_error_m": height_error,
                        "tracking_entry_before_pose_blend": "PASS" if joint_error <= core.config["entry"]["max_joint_error_rad"] and height_error <= core.config["entry"]["max_height_error_m"] else "POSE_BLEND_REQUIRED",
                        "pose_preparation_gate": "AVAILABLE" if int(low.mode_machine)==1 and int(low.mode_pr)==0 else "MODE_MISMATCH",
                        "entry_gate_applies_after_pose_preparation": True,
                        "observation_size": int(obs.size), "action_size": int(action.size),
                        "max_abs_policy_action": max_abs_action,
                        "lowstate_age_s": time.monotonic()-low_at, "estimator_age_s": time.monotonic()-est_at,
                    }
            time.sleep(.005)
            if unique >= max(5, int(args.seconds*40)):
                break
    finally:
        low_sub.Close(); est_sub.Close(); lowcmd_sub.Close()
    result = {"read_only": True, "motor_commands_sent": 0, "interface": args.interface,
              "unique_lowstate_samples": unique, "bad_lowstate_crc": bad_crc,
              "motion_switcher": {"result_code": int(owner_code), "owner": owner},
              "lowcmd_samples_during_preflight": lowcmd_samples[0],
              "another_lowcmd_writer_observed": lowcmd_samples[0] > 0, "latest": last_report}
    rendered = json.dumps(result, indent=2)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + "\n")
    return 0 if (unique >= max(5, int(args.seconds*40)) and bad_crc == 0 and last_report
                 and owner_code == 0 and owner is not None and not owner.get("name")
                 and lowcmd_samples[0] == 0) else 2


if __name__ == "__main__":
    raise SystemExit(main())
