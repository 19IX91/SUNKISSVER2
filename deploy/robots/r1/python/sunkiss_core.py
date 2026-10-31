"""Python Sunkiss R1 policy core, ported from r1_tracking_core.h.

This module only loads the reference/model and computes observations, actions,
and targets. It has no SDK imports and cannot send robot commands.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import onnxruntime as ort
import yaml

MOTOR_IDS = (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15, 16, 17, 18, 19, 22, 23, 24, 25, 26)
HEAD_IDS = (29, 30)


def _vec(value, shape, name):
    a = np.asarray(value, dtype=np.float32)
    if a.shape != shape or not np.isfinite(a).all():
        raise ValueError(f"{name} must be finite with shape {shape}; got {a.shape}")
    return a


def quat_normalize(q):
    q = _vec(q, (4,), "quaternion").astype(np.float64)
    n = np.linalg.norm(q)
    if not .99 <= n <= 1.01:
        raise ValueError(f"quaternion norm outside tolerance: {n}")
    return (q / n).astype(np.float32)


def quat_mul(a, b):
    aw, ax, ay, az = a; bw, bx, by, bz = b
    return np.array([aw*bw-ax*bx-ay*by-az*bz,
                     aw*bx+ax*bw+ay*bz-az*by,
                     aw*by-ax*bz+ay*bw+az*bx,
                     aw*bz+ax*by-ay*bx+az*bw], dtype=np.float32)


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float32)


def quat_matrix(q):
    w, x, y, z = quat_normalize(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]], dtype=np.float32)


def quat_yaw(q):
    r = quat_matrix(q)
    a = np.arctan2(r[1, 0], r[0, 0]) / 2
    return np.array([np.cos(a), 0, 0, np.sin(a)], dtype=np.float32)


def torso(state):
    roll_a = state.q[12] / 2
    turn_a = state.q[13] / 2
    roll = np.array([np.cos(roll_a), np.sin(roll_a), 0, 0], dtype=np.float32)
    turn = np.array([np.cos(turn_a), 0, 0, np.sin(turn_a)], dtype=np.float32)
    q_torso = quat_mul(quat_mul(state.orientation, roll), turn)
    offset = quat_matrix(quat_mul(state.orientation, roll)) @ np.array([.0325, 0, .049], dtype=np.float32)
    return state.position + offset, quat_normalize(q_torso)


@dataclass(frozen=True)
class State:
    position: np.ndarray
    orientation: np.ndarray
    velocity_body: np.ndarray
    angular_velocity_body: np.ndarray
    q: np.ndarray
    dq: np.ndarray

    def checked(self):
        return State(_vec(self.position, (3,), "position"), quat_normalize(self.orientation),
                     _vec(self.velocity_body, (3,), "body velocity"),
                     _vec(self.angular_velocity_body, (3,), "body gyro"),
                     _vec(self.q, (24,), "joint position"), _vec(self.dq, (24,), "joint velocity"))


@dataclass(frozen=True)
class Frame:
    q: np.ndarray
    dq: np.ndarray
    position: np.ndarray
    orientation: np.ndarray


class SunkissCore:
    """Numerically mirrors the Sunkiss C++ engine's 50 Hz policy path."""
    def __init__(self, deploy_yaml: Path):
        self.config = yaml.safe_load(Path(deploy_yaml).read_text())
        c = self.config
        if c.get("controller") != "r1_motion_tracking_v1":
            raise ValueError("wrong controller config")
        if tuple(c["joint_ids_map"]) != MOTOR_IDS or tuple(c["head_motor_ids"]) != HEAD_IDS:
            raise ValueError("R1 motor map mismatch")
        self.dt = float(c["step_dt"])
        if abs(self.dt - .02) > 1e-7:
            raise ValueError("policy period must be 50 Hz")
        self.offset = _vec(c["default_joint_pos"], (24,), "joint offset")
        self.scale = _vec(c["actions"]["scale"], (24,), "action scale")
        self.lower = _vec(c["joint_limits_lower"], (24,), "joint lower limits")
        self.upper = _vec(c["joint_limits_upper"], (24,), "joint upper limits")
        if np.any(self.lower >= self.upper) or np.any(self.scale <= 0):
            raise ValueError("invalid joint/action limits")
        self.reference = self._load_reference(Path(deploy_yaml).parent / c["motion_file"])
        if len(self.reference) != int(c["reference_frames"]):
            raise ValueError("reference frame count mismatch")
        base = Path(deploy_yaml).parent
        model = base / c["policy_file"]
        self.session = ort.InferenceSession(str(model), providers=["CPUExecutionProvider"])
        ins, outs = self.session.get_inputs(), self.session.get_outputs()
        if len(ins) != 1 or len(outs) != 1 or ins[0].shape != [1, 135] or outs[0].shape != [1, 24]:
            raise ValueError(f"bad ONNX contract: {ins[0].shape if ins else None} -> {outs[0].shape if outs else None}")
        if ins[0].name != "obs" or outs[0].name != "actions":
            raise ValueError("ONNX tensor names mismatch")
        self.input_name, self.output_name = ins[0].name, outs[0].name
        self.alignment = np.array([1, 0, 0, 0], dtype=np.float32)
        self.translation = np.zeros(3, dtype=np.float32)

    @staticmethod
    def _load_reference(path):
        raw = Path(path).read_bytes()
        if len(raw) < 16:
            raise ValueError("truncated reference header")
        magic, count, width = struct.unpack_from("<8sII", raw)
        if magic != b"R1REF001" or width != 55 or count < 2 or len(raw) != 16 + count*width*4:
            raise ValueError("invalid/truncated Sunkiss reference")
        rows = np.frombuffer(raw, dtype="<f4", offset=16).reshape(count, width).astype(np.float32, copy=True)
        if not np.isfinite(rows).all():
            raise ValueError("reference contains nonfinite values")
        qs = rows[:, 51:55]
        norms = np.linalg.norm(qs, axis=1)
        if np.any(np.abs(norms-1) >= .001):
            raise ValueError("reference has invalid quaternions")
        return tuple(Frame(r[:24].copy(), r[24:48].copy(), r[48:51].copy(), quat_normalize(r[51:55])) for r in rows)

    def align(self, state: State):
        s = state.checked(); p, q = torso(s); ref0 = self.reference[0]
        self.alignment = quat_yaw(quat_mul(q, quat_conj(ref0.orientation)))
        self.translation = p - quat_matrix(self.alignment) @ ref0.position
        self.translation[2] = 0.

    def transformed_frame(self, frame_index: int):
        f = self.reference[frame_index]
        return Frame(f.q, f.dq, quat_matrix(self.alignment) @ f.position + self.translation,
                     quat_normalize(quat_mul(self.alignment, f.orientation)))

    def observation(self, frame_index: int, state: State, last_action):
        s = state.checked(); last = _vec(last_action, (24,), "last action")
        f = self.transformed_frame(frame_index); p, q = torso(s)
        pos_b = quat_matrix(q).T @ (f.position - p)
        rel_q = quat_normalize(quat_mul(quat_conj(q), f.orientation))
        rot6 = quat_matrix(rel_q)[:, :2].reshape(6)
        obs = np.concatenate((f.q, f.dq, pos_b, rot6, s.velocity_body, s.angular_velocity_body,
                              s.q-self.offset, s.dq, last)).astype(np.float32)
        if obs.shape != (135,) or not np.isfinite(obs).all():
            raise ValueError("invalid 135-value observation")
        return obs

    def infer(self, observation):
        obs = _vec(observation, (135,), "observation")
        a = self.session.run([self.output_name], {self.input_name: obs[None, :]})[0][0]
        return _vec(a, (24,), "policy action")

    def target(self, action):
        return self.offset + self.scale * _vec(action, (24,), "policy action")

    def entry_error(self, state):
        s = state.checked(); p, _ = torso(s); f = self.reference[0]
        return float(np.max(np.abs(s.q-f.q))), float(abs(p[2]-f.position[2]))

    def check_entry(self, state):
        joint, height = self.entry_error(state)
        limits = self.config["entry"]
        if joint > float(limits["max_joint_error_rad"]):
            raise ValueError(f"first-reference joint error {joint:.3f} rad exceeds gate")
        if height > float(limits["max_height_error_m"]):
            raise ValueError(f"first-reference height error {height:.3f} m exceeds gate")
        return joint, height
