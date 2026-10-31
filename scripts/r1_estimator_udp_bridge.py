#!/usr/bin/env python3
"""Forward live R1 pelvis estimates to the local Sunkiss controller.

This process only subscribes to state topics and sends localhost UDP packets;
it never creates an rt/lowcmd publisher.
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "deploy/robots/r1/python"))
from estimator_data_idl import EstimatorData_


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--port", type=int, default=15150)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--max-age", type=float, default=0.05)
    args = parser.parse_args()
    if not (0 < args.port < 65536 and 0 < args.rate <= 100 and 0 < args.max_age <= 0.1):
        raise SystemExit("invalid bridge port/rate/max-age")

    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    except ImportError as exc:
        raise SystemExit("Unitree SDK2 Python is required") from exc

    lock = threading.Lock()
    latest: dict[str, object] = {"low": None, "est": None, "low_rx": 0.0, "est_rx": 0.0}

    def on_low(message) -> None:
        with lock:
            latest["low"] = message
            latest["low_rx"] = time.monotonic()

    def on_est(message) -> None:
        with lock:
            latest["est"] = message
            latest["est_rx"] = time.monotonic()

    ChannelFactoryInitialize(0, args.interface)
    low_sub = ChannelSubscriber("rt/lowstate", LowState_)
    est_sub = ChannelSubscriber("rt/EstimatorData", EstimatorData_)
    low_sub.Init(on_low, 10)
    est_sub.Init(on_est, 10)

    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    endpoint = ("127.0.0.1", args.port)
    period = 1.0 / args.rate
    last_stamp = 0
    sent = 0
    print(f"Sunkiss state bridge listening on rt/lowstate + rt/EstimatorData; UDP {endpoint}", flush=True)
    try:
        while True:
            started = time.monotonic()
            with lock:
                low, est = latest["low"], latest["est"]
                low_rx, est_rx = float(latest["low_rx"]), float(latest["est_rx"])
            now = time.monotonic()
            if low is None or est is None or now-low_rx > args.max_age or now-est_rx > args.max_age:
                print("state feed stale/missing; withholding estimates", flush=True)
                time.sleep(period)
                continue

            # R1 EstimatorData.position_2d is [world x, world y, body height].
            # position_3d.z is not pelvis height on this firmware.
            position = np.asarray(est.position_2d, dtype=np.float64)
            velocity = np.asarray(est.v_body_3d, dtype=np.float64)
            quaternion = np.asarray(low.imu_state.quaternion, dtype=np.float64)
            norm = float(np.linalg.norm(quaternion))
            if position.shape != (3,) or velocity.shape != (3,) or quaternion.shape != (4,) or not np.isfinite(np.r_[position, velocity, quaternion]).all() or not 0.5 < position[2] < 1.2 or not 0.99 < norm < 1.01:
                print("invalid pelvis estimate; withholding packet", flush=True)
                time.sleep(period)
                continue
            quaternion /= norm
            stamp = int(min(low_rx, est_rx) * 1e9)
            if stamp <= last_stamp:
                time.sleep(period)
                continue
            packet = {
                "version": 1,
                "frame": "pelvis",
                "stamp_ns": stamp,
                "position_w": position.tolist(),
                "quaternion_wxyz": quaternion.tolist(),
                "linear_velocity_b": velocity.tolist(),
            }
            sender.sendto(json.dumps(packet, separators=(",", ":")).encode(), endpoint)
            last_stamp = stamp
            sent += 1
            if sent == 1 or sent % int(args.rate) == 0:
                print(f"sent={sent} height={position[2]:.3f}m body_speed={np.linalg.norm(velocity):.3f}m/s", flush=True)
            time.sleep(max(0.0, period - (time.monotonic() - started)))
    except KeyboardInterrupt:
        print(f"state bridge stopped after {sent} packets", flush=True)
    finally:
        sender.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
