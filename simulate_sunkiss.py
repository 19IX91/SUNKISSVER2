#!/usr/bin/env python3
"""Independent Sunkiss MuJoCo replay using simulated pelvis ground truth.

This is an approximation using the local R1 URDF model. It does not connect to
hardware or establish equivalence with the source training simulator.
"""
import argparse
import csv
import json
import math
import struct
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np
import onnxruntime as ort
import yaml
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'deploy/robots/r1/python'))
from sunkiss_core import SunkissCore, State as SunkissState
CFG = ROOT / 'deploy/robots/r1/config/policy/tracking/sunkiss/v1/params/deploy.yaml'
MODEL = ROOT / 'mujoco/r1_walk_actuated.xml'

def xyzw(wxyz):
    return np.roll(wxyz, -1)

def wxyz(rotation):
    return np.roll(rotation.as_quat(), 1)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'mujoco_result.json')
    parser.add_argument('--config', type=Path, default=CFG, help='deployment YAML (may point to an alternate reference)')
    parser.add_argument('--model', type=Path, default=MODEL)
    parser.add_argument('--reference-velocity', action='store_true', help='Initialize joints at reference velocities for diagnostic comparison')
    parser.add_argument('--cpp-engine', action='store_true', help='Drive simulation with supplied C++ engine and compare observations')
    parser.add_argument('--initial-z-offset', type=float, default=0., help='Pelvis height perturbation in meters')
    parser.add_argument('--physics-dt', type=float, default=.002, help='MuJoCo physics timestep in seconds')
    parser.add_argument('--gain-scale', type=float, default=1.0, help='multiply simulated joint kp and kd for sensitivity analysis only')
    parser.add_argument('--video', type=Path, help='Save the simulated policy replay as MP4 (use MUJOCO_GL=egl)')
    parser.add_argument('--trace-csv', type=Path, help='Write one measured MuJoCo state row per policy frame')
    parser.add_argument('--world-yaw-deg', type=float, default=0., help='Rotate starting pose around world vertical')
    parser.add_argument('--world-x', type=float, default=0.)
    parser.add_argument('--world-y', type=float, default=0.)
    args = parser.parse_args()
    if not np.isfinite(args.gain_scale) or args.gain_scale <= 0:
        parser.error('--gain-scale must be finite and positive')
    config_path = args.config.resolve()
    cfg = yaml.safe_load(config_path.read_text())
    with (config_path.parent / cfg['motion_file']).open('rb') as f:
        magic, count, width = struct.unpack('<8sII', f.read(16))
        if magic != b'R1REF001' or width != 55 or count != cfg['reference_frames']:
            raise ValueError('invalid reference header')
        ref = np.frombuffer(f.read(), dtype='<f4').reshape(count, width).copy()
    m = mujoco.MjModel.from_xml_path(str(args.model))
    m.opt.timestep = args.physics_dt
    steps = round(cfg['step_dt'] / m.opt.timestep)
    if steps < 1 or abs(steps * m.opt.timestep - cfg['step_dt']) > 1e-9: raise ValueError('physics timestep must divide policy timestep')
    if abs(args.physics_dt - .005) < 1e-9:
        m.opt.iterations = 10; m.opt.ls_iterations = 20
    m.actuator_gainprm[:, 0] = 0
    m.actuator_biasprm[:, :] = 0
    d = mujoco.MjData(m)
    names = cfg['joint_names']
    ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in names]
    if min(ids) < 0: raise ValueError('joint missing from MuJoCo model')
    qadr, vadr = m.jnt_qposadr[ids], m.jnt_dofadr[ids]
    pelvis = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'pelvis_link')
    if pelvis < 0: pelvis = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'pelvis')
    if pelvis < 0: raise ValueError('pelvis missing')
    q0 = ref[0, :24]
    roll = Rotation.from_rotvec([q0[12], 0, 0])
    turn = Rotation.from_rotvec([0, 0, q0[13]])
    pelvis_rot = Rotation.from_quat(xyzw(ref[0, 51:55])) * (roll * turn).inv()
    pelvis_pos = ref[0, 48:51] - pelvis_rot.apply(roll.apply([.0325, 0, .049]))
    world_rot=Rotation.from_euler('z',args.world_yaw_deg,degrees=True)
    pelvis_pos=world_rot.apply(pelvis_pos)+np.array([args.world_x,args.world_y,args.initial_z_offset])
    pelvis_rot=world_rot*pelvis_rot
    d.qpos[:3], d.qpos[3:7], d.qpos[qadr] = pelvis_pos, wxyz(pelvis_rot), q0
    if args.reference_velocity:
        d.qvel[vadr] = ref[0, 24:48]
        q1 = ref[1,:24]
        roll1 = Rotation.from_rotvec([q1[12],0,0])
        turn1 = Rotation.from_rotvec([0,0,q1[13]])
        rot1 = Rotation.from_quat(xyzw(ref[1,51:55])) * (roll1 * turn1).inv()
        pos1 = ref[1,48:51] - rot1.apply(roll1.apply([.0325,0,.049]))
        pos1=world_rot.apply(pos1)+np.array([args.world_x,args.world_y,args.initial_z_offset]);rot1=world_rot*rot1
        d.qvel[:3] = (pos1 - pelvis_pos) / cfg['step_dt']
        d.qvel[3:6] = (pelvis_rot.inv() * rot1).as_rotvec() / cfg['step_dt']
    mujoco.mj_forward(m, d)
    def torso():
        base = Rotation.from_quat(xyzw(d.xquat[pelvis]))
        roll = Rotation.from_rotvec([d.qpos[qadr[12]], 0, 0])
        turn = Rotation.from_rotvec([0, 0, d.qpos[qadr[13]]])
        return d.xpos[pelvis] + base.apply(roll.apply([.0325, 0, .049])), base * roll * turn
    p, r = torso()
    delta = r * Rotation.from_quat(xyzw(ref[0, 51:55])).inv()
    align = Rotation.from_euler('z', math.atan2(delta.as_matrix()[1,0],delta.as_matrix()[0,0]))
    translation = p - align.apply(ref[0,48:51]); translation[2] = 0
    offset = np.array(cfg['default_joint_pos']); scale = np.array(cfg['actions']['scale'])
    kp, kd, effort = (np.array(cfg[k]) for k in ('stiffness','damping','effort_limits'))
    kp *= args.gain_scale
    kd *= args.gain_scale
    core = SunkissCore(config_path)
    last = np.zeros(24); max_tilt = 0.; max_tilt_time = 0.; max_ori_error = 0.; max_height_error = 0.; max_joint_error = 0.; max_effort_ratio = 0.; completed = 0; reason = 'completed'
    min_pelvis_height=float(d.qpos[2]); max_xy_error=0.; floor_contact_frames=0; nonfoot_contacts=set(); max_speed=0.; max_speed_joint='';max_speed_time=0.; measured_limit_violation=None
    velocity = np.zeros(6)
    writer = renderer = None
    if args.video:
        import cv2
        renderer=mujoco.Renderer(m,480,640)
        writer=cv2.VideoWriter(str(args.video),cv2.VideoWriter_fourcc(*'mp4v'),50,(640,480))
        if not writer.isOpened(): raise RuntimeError('Cannot open video writer')
        camera=mujoco.MjvCamera(); camera.type=mujoco.mjtCamera.mjCAMERA_FREE; camera.distance=2.4; camera.azimuth=135; camera.elevation=-12
    cpp = subprocess.Popen([str(ROOT / 'r1_tracking_offline'), str(config_path)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1) if args.cpp_engine else None
    max_obs_diff = 0.; max_action_diff = 0.
    trace_file = args.trace_csv.open('w', newline='') if args.trace_csv else None
    trace = csv.writer(trace_file) if trace_file else None
    if trace: trace.writerow(['time_s','pelvis_tilt_deg','pelvis_z_m','pelvis_x_m','pelvis_y_m','reference_x_m','reference_y_m','anchor_xy_error_m','anchor_height_error_m','anchor_orientation_error_rad','max_abs_joint_speed_rad_s','max_abs_reference_joint_speed_rad_s','nonfoot_contact_count'])
    for frame in range(count-1):
        mujoco.mj_forward(m,d)
        mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_XBODY,pelvis,velocity,1)
        if renderer:
            camera.lookat[:] = [d.qpos[0],d.qpos[1],.65]
            renderer.update_scene(d,camera=camera)
            picture=cv2.cvtColor(renderer.render(),cv2.COLOR_RGB2BGR)
            cv2.putText(picture, f'POLICY REPLAY  t={d.time:.2f}s', (12,28), cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),2)
            writer.write(picture)
        p,r = torso(); target_ref = ref[frame]
        err_rot = r.inv() * align * Rotation.from_quat(xyzw(target_ref[51:55]))
        anchor = r.inv().apply(align.apply(target_ref[48:51]) + translation - p)
        py_state = SunkissState(d.xpos[pelvis].astype(np.float32).copy(),
                                d.xquat[pelvis].astype(np.float32).copy(),
                                velocity[3:].astype(np.float32).copy(),
                                velocity[:3].astype(np.float32).copy(),
                                d.qpos[qadr].astype(np.float32).copy(),
                                d.qvel[vadr].astype(np.float32).copy())
        if frame == 0:
            core.align(py_state)
        obs = core.observation(frame, py_state, last)
        if obs.shape != (135,) or not np.isfinite(obs).all(): reason='invalid observation';break
        py_action = core.infer(obs)
        if cpp:
            state={'frame':frame,'align':frame==0,'position_w':d.xpos[pelvis].tolist(),'quaternion_wxyz':d.xquat[pelvis].tolist(),'linear_velocity_b':velocity[3:].tolist(),'angular_velocity_b':velocity[:3].tolist(),'joint_pos':d.qpos[qadr].tolist(),'joint_vel':d.qvel[vadr].tolist(),'last_action':last.tolist()}
            cpp.stdin.write(json.dumps(state)+'\n'); cpp.stdin.flush()
            line=cpp.stdout.readline()
            if not line: raise RuntimeError('C++ tracking engine exited')
            result=json.loads(line)
            if 'error' in result: raise RuntimeError('C++ tracking engine: '+result['error'])
            max_obs_diff=max(max_obs_diff,float(np.max(np.abs(obs-np.array(result['obs'])))))
            max_action_diff=max(max_action_diff,float(np.max(np.abs(py_action-np.array(result['actions'])))))
            last=py_action;target=core.target(py_action)
        else:
            last=py_action;target=core.target(py_action)
        if not np.isfinite(target).all(): reason='invalid target';break
        height_error = abs(p[2]-target_ref[50]); ori_error = err_rot.magnitude()
        min_pelvis_height=min(min_pelvis_height,float(d.qpos[2]));max_xy_error=max(max_xy_error,float(np.linalg.norm(anchor[:2])))
        speed_idx=int(np.argmax(np.abs(d.qvel[vadr])))
        if abs(d.qvel[vadr[speed_idx]])>max_speed:
            max_speed=float(abs(d.qvel[vadr[speed_idx]]));max_speed_joint=names[speed_idx];max_speed_time=float(d.time)
        lower=np.array(cfg['joint_limits_lower']);upper=np.array(cfg['joint_limits_upper'])
        violations=np.where((d.qpos[qadr]<lower-.1)|(d.qpos[qadr]>upper+.1))[0]
        if len(violations):
            i=int(violations[0]);measured_limit_violation={'joint':names[i],'position_rad':float(d.qpos[qadr[i]]),'time_s':float(d.time)}
            reason='measured joint limit guard exceeded';break
        for contact in d.contact[:d.ncon]:
            g1=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,contact.geom1) or ''
            g2=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,contact.geom2) or ''
            if 'floor' in (g1,g2):
                floor_contact_frames+=1
                other=g2 if g1=='floor' else g1
                if '_foot' not in other: nonfoot_contacts.add(other or '<unnamed>')
        tilt = math.acos(np.clip(Rotation.from_quat(xyzw(d.xquat[pelvis])).as_matrix()[2,2],-1,1))
        if tilt > max_tilt: max_tilt=tilt; max_tilt_time=float(d.time)
        max_ori_error=max(max_ori_error,ori_error);max_height_error=max(max_height_error,height_error)
        max_joint_error=max(max_joint_error, float(np.max(np.abs(d.qpos[qadr]-target_ref[:24]))))
        if trace:
            trace.writerow([float(d.time),math.degrees(tilt),float(d.qpos[2]),float(p[0]),float(p[1]),float(target_ref[48]),float(target_ref[49]),float(np.linalg.norm(anchor[:2])),float(height_error),float(ori_error),float(np.max(np.abs(d.qvel[vadr]))),float(np.max(np.abs(target_ref[24:48]))),len(nonfoot_contacts)])
        if height_error > cfg['stop']['max_anchor_height_error_m'] or ori_error > cfg['stop']['max_anchor_orientation_error_rad']:
            reason='tracking guard exceeded';break
        for _ in range(steps):
            torque=kp*(target-d.qpos[qadr])-kd*d.qvel[vadr]
            max_effort_ratio=max(max_effort_ratio,float(np.max(np.abs(torque)/effort)))
            d.qfrc_applied[vadr]=np.clip(torque,-effort,effort)
            mujoco.mj_step(m,d)
        completed += 1
    if cpp:
        cpp.stdin.close(); cpp.wait(timeout=5)
    if renderer:
        renderer.close(); writer.release()
    if trace_file: trace_file.close()
    result={'cpp_engine_used':bool(cpp),'max_cpp_python_observation_difference':max_obs_diff if cpp else None,'max_cpp_python_action_difference':max_action_diff if cpp else None,'model':str(args.model),'mode':'Python SunkissCore with simulated pelvis ground truth'+(' and C++ parity comparison' if cpp else ''), 'world_yaw_deg':args.world_yaw_deg,'world_xy':[args.world_x,args.world_y],'simulated_gain_scale':args.gain_scale,'min_pelvis_height_m':min_pelvis_height,'max_anchor_xy_error_m':max_xy_error,'max_joint_speed_rad_s':max_speed,'max_joint_speed_joint':max_speed_joint,'max_joint_speed_time_s':max_speed_time,'measured_limit_violation':measured_limit_violation,'nonfoot_floor_contacts':sorted(nonfoot_contacts),'physics_timestep_s':args.physics_dt,'initial_z_offset_m':args.initial_z_offset,'initialization':'first reference pose with '+('reference joint and finite-difference base velocities' if args.reference_velocity else 'zero velocities'), 'reference_frames':count,'steps_completed':completed,'duration_s':round(float(d.time),3),'reason':reason,'max_pelvis_tilt_deg':round(math.degrees(max_tilt),3),'max_pelvis_tilt_time_s':max_tilt_time,'max_anchor_orientation_error_rad':round(max_ori_error,4),'max_anchor_height_error_m':round(max_height_error,4),'max_joint_reference_error_rad':round(max_joint_error,4),'max_unclipped_effort_ratio':round(max_effort_ratio,4),'final_pelvis_position':d.qpos[:3].tolist()}
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return 0 if reason=='completed' else 1

if __name__ == '__main__':
    raise SystemExit(main())
