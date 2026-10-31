#!/usr/bin/env python3
"""Supervised Python Sunkiss controller for R1 EDU.

No DDS command publisher is created without both --arm and the exact live
confirmation. Startup follows the R1 walking deployment pattern: read-only
checks, operator pose-preparation trigger, five-second smooth pose blend,
settle/start gate, handheld policy start, and watchdogged lowcmd publishing.
"""
from __future__ import annotations

import argparse
import fcntl
import json
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

from sunkiss_core import MOTOR_IDS, HEAD_IDS, SunkissCore, State, quat_matrix, quat_mul, quat_conj

R1_ROOT = Path(__file__).resolve().parents[1]
CONFIG = R1_ROOT / "config/policy/tracking/sunkiss/v1/params/deploy.yaml"


def buttons(remote):
    return {"prepare": bool(remote[2] & 0x20 and remote[3] & 0x10),
            "start": bool(remote[2] & 0x10 and remote[3] & 0x01),
            "stop": bool(remote[2] & 0x20 and remote[3] & 0x02)}


def tilt_deg(q):
    return math.degrees(math.acos(float(np.clip(quat_matrix(q)[2, 2], -1., 1.))))


def build_lowcmd(command, cfg, mode_machine, measured_q, target_q, head_q, enabled=True):
    """Fill a Unitree R1 lowcmd message without publishing it."""
    measured_q=np.asarray(measured_q,np.float32); target_q=np.asarray(target_q,np.float32); head_q=np.asarray(head_q,np.float32)
    if measured_q.shape!=(24,) or target_q.shape!=(24,) or head_q.shape!=(2,):
        raise ValueError("invalid lowcmd vector sizes")
    command.mode_machine=int(mode_machine); command.mode_pr=0
    active={i:j for j,i in enumerate(MOTOR_IDS)}
    for i,m in enumerate(command.motor_cmd):
        if i in active:
            j=active[i]; m.mode=1; m.q=float(target_q[j]) if enabled else float(measured_q[j])
            m.kp=float(cfg["stiffness"][j]) if enabled else 0.; m.kd=float(cfg["damping"][j]) if enabled else 3.
            m.dq=0.; m.tau=0.
        elif i in HEAD_IDS:
            j=HEAD_IDS.index(i); m.mode=1; m.q=float(head_q[j]); m.kp=5. if enabled else 0.; m.kd=1. if enabled else 3.
            m.dq=0.; m.tau=0.
        else:
            m.mode=0; m.q=m.kp=m.kd=m.dq=m.tau=0.
    return command


class HardwareIO:
    """Fresh CRC-checked lowstate plus the robot's EstimatorData feed."""
    def __init__(self, interface):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
        from unitree_sdk2py.utils.crc import CRC
        from estimator_data_idl import EstimatorData_
        ChannelFactoryInitialize(0, interface)
        self.lock = threading.Lock(); self.low = self.est = None
        self.low_at = self.est_at = 0.; self.low_count = self.est_count = 0
        self.crc = CRC()
        def on_low(m):
            with self.lock: self.low, self.low_at, self.low_count = m, time.monotonic(), self.low_count+1
        def on_est(m):
            with self.lock: self.est, self.est_at, self.est_count = m, time.monotonic(), self.est_count+1
        self.lowcmd_count = 0
        self.lowcmd_sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
        self.lowcmd_sub.Init(lambda _m: self._count_lowcmd(), 1)
        self.low_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.est_sub = ChannelSubscriber("rt/EstimatorData", EstimatorData_)
        self.low_sub.Init(on_low, 10); self.est_sub.Init(on_est, 10)

    def snapshot(self, max_age=.1):
        with self.lock:
            low, est, la, ea = self.low, self.est, self.low_at, self.est_at
        now = time.monotonic()
        if low is None or est is None or now-la > max_age or now-ea > max_age:
            raise RuntimeError("lowstate/EstimatorData missing or stale")
        if self.crc.Crc(low) != low.crc: raise RuntimeError("lowstate CRC mismatch")
        if int(low.mode_machine) != 1 or int(low.mode_pr) != 0:
            raise RuntimeError(f"expected low-level R1 mode 1/0; got {low.mode_machine}/{low.mode_pr}")
        if len(low.motor_state) < 31: raise RuntimeError("R1 lowstate has fewer than 31 motors")
        q = np.asarray([low.motor_state[i].q for i in MOTOR_IDS], np.float32)
        dq = np.asarray([low.motor_state[i].dq for i in MOTOR_IDS], np.float32)
        faults = [(i, int(low.motor_state[i].motorstate)) for i in MOTOR_IDS if low.motor_state[i].motorstate]
        if faults: raise RuntimeError(f"motor error flags: {faults}")
        s = State(np.asarray(est.position_2d, np.float32), np.asarray(low.imu_state.quaternion, np.float32),
                  np.asarray(est.v_body_3d, np.float32), np.asarray(low.imu_state.gyroscope, np.float32), q, dq).checked()
        if not np.isfinite(s.position).all() or not .35 < float(s.position[2]) < 1.2:
            raise RuntimeError("invalid pelvis estimate/height")
        return s, low, buttons(low.wireless_remote), tilt_deg(s.orientation), now-la, now-ea

    def _count_lowcmd(self):
        with self.lock: self.lowcmd_count += 1

    def ensure_lowcmd_quiet(self):
        with self.lock: before=self.lowcmd_count
        time.sleep(.5)
        with self.lock: after=self.lowcmd_count
        if after != before: raise RuntimeError("another rt/lowcmd publisher is active")

    def close(self): self.low_sub.Close(); self.est_sub.Close(); self.lowcmd_sub.Close()


class LowCommandWriter:
    """500 Hz SDK publisher; only trained body slots and head hold are enabled."""
    def __init__(self, io, cfg, initial_q, initial_head):
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.utils.crc import CRC
        self.io, self.cfg = io, cfg
        self.command = unitree_hg_msg_dds__LowCmd_(); self.crc = CRC()
        self.publisher = ChannelPublisher("rt/lowcmd", LowCmd_); self.publisher.Init()
        self.lock = threading.Lock(); self.target = np.asarray(initial_q,np.float32).copy()
        self.head = np.asarray(initial_head,np.float32).copy(); self.kp_scale = 1.; self.stop = threading.Event()
        self.fault = None; self.sent = 0; self.last_write_ok = 0
        self.thread = threading.Thread(target=self._run, name="sunkiss-lowcmd-500hz", daemon=True)
        self.thread.start()

    def set(self, q24, head2, gains=True):
        q24 = np.asarray(q24, np.float32); head2 = np.asarray(head2, np.float32)
        if q24.shape != (24,) or head2.shape != (2,) or not np.isfinite(q24).all() or not np.isfinite(head2).all():
            raise ValueError("invalid command target")
        with self.lock: self.target=q24.copy(); self.head=head2.copy(); self.kp_scale=1. if gains else 0.

    def _run(self):
        period=.002; deadline=time.monotonic()
        while not self.stop.is_set():
            try:
                s, low, _, _, _, _ = self.io.snapshot()
                with self.lock: target=self.target.copy(); head=self.head.copy(); kp_scale=self.kp_scale
                build_lowcmd(self.command,self.cfg,low.mode_machine,s.q,target,head,bool(kp_scale))
                self.command.crc=self.crc.Crc(self.command)
                ok=self.publisher.Write(self.command, timeout=.002)
                self.last_write_ok = time.monotonic()
                if ok: self.sent+=1
                else: self.trip("DDS write failed/no lowcmd reader")
            except Exception as e:
                self.trip(f"writer watchdog: {e}")
            deadline += period; remaining=deadline-time.monotonic()
            if remaining>0: self.stop.wait(remaining)
            else: deadline=time.monotonic()

    def trip(self, reason):
        self.fault = self.fault or reason
        with self.lock: self.kp_scale=0.

    def close(self):
        self.trip("operator shutdown/damping")
        time.sleep(.3); self.stop.set(); self.thread.join(timeout=1.)
        self.publisher.Close()


def check_owner():
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    c=MotionSwitcherClient(); c.SetTimeout(2.); c.Init(); code,result=c.CheckMode()
    if code or result is None: raise RuntimeError(f"MotionSwitcher check failed ({code})")
    if result.get("name"): raise RuntimeError(f"motion service owner must be empty before low-level control: {result}")


def acquire_process_lock():
    path=Path("/tmp/r1_sunkiss_python.lock")
    handle=path.open("a+")
    try: fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        handle.close(); raise RuntimeError("another Python Sunkiss controller is already running") from None
    handle.seek(0); handle.truncate(); handle.write(str(os.getpid())); handle.flush()
    return handle


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--interface", help="R1 Ethernet interface; required for --check/--arm")
    ap.add_argument("--check", action="store_true", help="read-only live policy/state check; no publisher")
    ap.add_argument("--offline-check", action="store_true", help="load package and exercise ONNX locally; no DDS/network")
    ap.add_argument("--arm", action="store_true", help="enable supervised handheld pose-prep and tracking")
    ap.add_argument("--confirm-live-sunkiss", action="store_true", help="required together with --arm")
    ap.add_argument("--pose-seconds", type=float, default=5.)
    ap.add_argument("--max-seconds", type=float, default=5., help="policy test duration (use 26.5 for the complete sequence)")
    a=ap.parse_args()
    if not 3.<=a.pose_seconds<=15. or not 0<a.max_seconds<=30: ap.error("invalid duration")
    if a.offline_check:
        if a.arm or a.check or a.confirm_live_sunkiss: ap.error("--offline-check cannot be combined with live options")
        core=SunkissCore(CONFIG)
        action=core.infer(np.zeros(135,np.float32)); target=core.target(action)
        if len(core.reference)!=int(core.config["reference_frames"]): ap.error("reference frame count mismatch")
        if not np.isfinite(target).all() or np.any(target<core.lower-.05) or np.any(target>core.upper+.05):
            ap.error("zero-observation policy smoke target failed configured joint limits")
        try:
            from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
            from unitree_sdk2py.utils.crc import CRC
            command=unitree_hg_msg_dds__LowCmd_()
            build_lowcmd(command,core.config,1,np.zeros(24,np.float32),target,np.zeros(2,np.float32))
            active_count=sum(int(m.mode)==1 for m in command.motor_cmd)
            command.crc=CRC().Crc(command)
            if active_count!=len(MOTOR_IDS)+len(HEAD_IDS) or int(command.mode_machine)!=1 or not command.crc:
                ap.error("Unitree lowcmd layout/CRC smoke check failed")
            damping=unitree_hg_msg_dds__LowCmd_()
            build_lowcmd(damping,core.config,1,np.ones(24,np.float32),target,np.ones(2,np.float32),False)
            if any(float(damping.motor_cmd[i].kp)!=0. or float(damping.motor_cmd[i].q)!=1. for i in MOTOR_IDS):
                ap.error("lowcmd damping layout smoke check failed")
        except ImportError as e:
            ap.error(f"Unitree SDK Python required for deployment: {e}")
        print(json.dumps({"result":"PASS","mode":"offline_only","reference_frames":len(core.reference),
                          "joint_map":list(MOTOR_IDS),"policy_input":135,"policy_output":24,
                          "action_finite":bool(np.isfinite(action).all()),"target_within_limits":True,
                          "lowcmd_motor_slots":active_count,"lowcmd_crc_generated":bool(command.crc),
                          "damping_message_layout":"PASS",
                          "dds_opened":False,"motor_commands_sent":0},indent=2))
        return 0
    if not a.interface: ap.error("--interface is required for live checks and control")
    if a.check and a.arm: ap.error("--check is read-only and cannot be combined with --arm")
    if a.arm != a.confirm_live_sunkiss: ap.error("live operation requires both --arm and --confirm-live-sunkiss")
    core=SunkissCore(CONFIG); cfg=core.config
    if np.any(core.reference[0].q < core.lower-.05) or np.any(core.reference[0].q > core.upper+.05):
        ap.error("first Sunkiss pose violates configured joint limits")
    if a.check and not a.arm:
        return subprocess.run([sys.executable, str(Path(__file__).with_name("sunkiss_hardware_preflight.py")),
                               "--interface", a.interface], check=False).returncode
    if not a.arm: ap.error("choose --check (read-only) or explicit --arm --confirm-live-sunkiss")
    # Subscribe first to initialize Cyclone DDS on the requested robot interface.
    process_lock=acquire_process_lock()
    io=HardwareIO(a.interface); writer=None
    try: check_owner()
    except Exception:
        io.close(); process_lock.close(); raise
    state="WAITING"; prev={"prepare":True,"start":True,"stop":False}
    start_q=head=None; stage_at=0.; frame=0; dance_at=0.; last_action=np.zeros(24,np.float32)
    next_policy=time.monotonic(); next_log=0.; fault=None
    print("ARMED in read-only WAITING. Hold LT+Up to begin the 5 s reference-pose blend. RT+A starts tracking; LT+B damps.",flush=True)
    try:
        while True:
            now=time.monotonic()
            s,low,b,tilt,la,ea=io.snapshot()
            if writer and writer.fault and state!="DAMPED": raise RuntimeError(writer.fault)
            if b["stop"] and writer:
                writer.trip("LT+B damping requested"); state="DAMPED"
            if state=="WAITING":
                if b["prepare"] and not prev["prepare"]:
                    # A nonmatching initial pose is expected; the operator's
                    # preparation request begins the blend without comparing q.
                    check_owner(); io.ensure_lowcmd_quiet()
                    start_q=s.q.copy(); head=np.asarray([low.motor_state[i].q for i in HEAD_IDS],np.float32)
                    writer=LowCommandWriter(io,cfg,start_q,head); state="PREPARE"; stage_at=now
                    print("PREPARE: smooth joint interpolation started",flush=True)
            elif state=="PREPARE":
                u=min((now-stage_at)/a.pose_seconds,1.); u=u*u*(3-2*u)
                target=(1-u)*start_q+u*core.reference[0].q
                writer.set(target,head)
                if u>=1.:
                    state="HOLD"; print("HOLD: settle in the reference pose; RT+A starts Sunkiss",flush=True)
            elif state=="HOLD":
                writer.set(core.reference[0].q,head)
                joint_err,height_err=core.entry_error(s)
                ready=(joint_err<=cfg["entry"]["max_joint_error_rad"] and height_err<=cfg["entry"]["max_height_error_m"] and np.max(np.abs(s.dq))<=1.)
                if b["start"] and not prev["start"]:
                    if not ready: raise RuntimeError(f"tracking entry gate failed: joint={joint_err:.3f} rad, height={height_err:.3f} m, max|dq|={np.max(np.abs(s.dq)):.3f}")
                    core.align(s); state="TRACK"; frame=0; last_action=np.zeros(24,np.float32); dance_at=now; next_policy=now
                    print("TRACK: Sunkiss started",flush=True)
            elif state=="TRACK":
                if now-dance_at>=min(a.max_seconds,len(core.reference)*core.dt):
                    writer.trip("dance duration complete"); state="DAMPED"
                elif now>=next_policy:
                    if now-next_policy>cfg["stop"]["max_policy_lateness_s"]: raise RuntimeError("policy deadline missed")
                    obs=core.observation(frame,s,last_action); action=core.infer(obs); target=core.target(action)
                    p,_=__import__("sunkiss_core").torso(s); ref=core.transformed_frame(frame)
                    if abs(float(p[2]-ref.position[2]))>cfg["stop"]["max_anchor_height_error_m"]: raise RuntimeError("pelvis height tracking guard")
                    _,qtorso=__import__("sunkiss_core").torso(s)
                    qerr=quat_mul(quat_conj(qtorso),ref.orientation)
                    angle=2*math.acos(float(np.clip(abs(qerr[0]),0,1)))
                    if angle>cfg["stop"]["max_anchor_orientation_error_rad"]: raise RuntimeError("pelvis orientation tracking guard")
                    if np.any(target < core.lower-.05) or np.any(target > core.upper+.05):
                        raise RuntimeError("policy target outside configured joint limits")
                    last_action=action; writer.set(target,head); frame=(frame+1)%len(core.reference)
                    next_policy+=core.dt
            elif state=="DAMPED":
                break
            if now>=next_log:
                print(json.dumps({"state":state,"tilt_deg":round(tilt,2),"joint_speed_max":round(float(np.max(np.abs(s.dq))),3),"joint_error":round(core.entry_error(s)[0],3),"lowstate_age_s":round(la,4),"estimator_age_s":round(ea,4),"commands":writer.sent if writer else 0}),flush=True)
                next_log=now+1.
            prev=b; time.sleep(.002)
    except KeyboardInterrupt:
        print("Keyboard interrupt: damping",flush=True)
    except Exception as e:
        fault=str(e); print(f"FAULT -> damping: {fault}",flush=True)
        if writer: writer.trip(fault)
    finally:
        if writer: writer.close()
        io.close()
        process_lock.close()
    return 1 if fault else 0


if __name__=="__main__":
    raise SystemExit(main())
