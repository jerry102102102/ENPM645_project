"""CPU-only contact pilot. No viewer, camera, label-to-sensor path, or pose scripting."""
from __future__ import annotations
import argparse
import hashlib
import json
import platform
import sys
from pathlib import Path
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'models/tactile_pilot.xml'
PIVOT = np.array([-.25, 0., .8])
HAND = np.array([.27, 0., 0.])


def rotate_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def clip_norm(v, limit):
    return v * min(1., limit / max(np.linalg.norm(v), 1e-12))


def smooth(t, start, duration):
    u = np.clip((t-start)/duration, 0., 1.)
    return u*u*(3-2*u), (6*u*(1-u)/duration if 0 < u < 1 else 0.)


def parameters(i, base_seed):
    rng = np.random.default_rng(base_seed+i)
    params = dict(seed=base_seed+i, block=i, kh=float(rng.uniform(65, 110)),
                dh=float(rng.uniform(4, 8)), friction=float(rng.uniform(.85, 1.05)),
                preload=40., initial_angle=float(rng.uniform(-.015, .015)),
                direction=1 if i % 2 == 0 else -1,
                target_angle=float(np.deg2rad(rng.uniform(15, 25))),
                ramp_duration=float(rng.uniform(.8, 1.3)),
                disturbance_magnitude=float(rng.uniform(7, 22)),
                disturbance_point=[float(rng.uniform(-.08, .18)), float(rng.uniform(-.035, .035)), 0.],
                disturbance_start=.7, disturbance_end=2.7,
                human_force_limit=50., human_target_speed_limit=.35,
                settle_seconds=1.5, duration=4., timestep=.002, sample_hz=100)
    params['ramp_duration'] = max(params['ramp_duration'], 1.5*.52*params['target_angle']/params['human_target_speed_limit'])
    return params


class Pilot:
    def __init__(self, p):
        self.p = p
        self.m = mujoco.MjModel.from_xml_path(str(MODEL))
        self.d = mujoco.MjData(self.m)
        m, d = self.m, self.d
        self.panel = m.body('panel').id
        self.pg = m.geom('panel_geom').id
        self.pad_g = [m.geom(x+'_pad').id for x in ('upper', 'lower')]
        self.pad_b = [m.body(x).id for x in ('upper', 'lower')]
        self.wrist = m.body('wrist').id
        self.hsite = m.site('human_grasp').id
        self.qadr = m.jnt_qposadr[m.joint('panel_free').id]
        self.vadr = m.jnt_dofadr[m.joint('panel_free').id]
        m.geom_friction[:, 0] = p['friction']
        # Initialization only: all subsequent motion is mj_step integration.
        a = p['initial_angle']
        d.qpos[0] = a
        d.qpos[self.qadr:self.qadr+3] = PIVOT + rotate_y(a) @ np.array([.25, 0, 0])
        d.qpos[self.qadr+3:self.qadr+7] = [np.cos(a/2), 0, np.sin(a/2), 0]
        d.ctrl[:] = [-p['preload'], p['preload']]
        mujoco.mj_forward(m, d)
        # Constant target offset supplies approximately half the panel's weight.
        self.base_hand = PIVOT + rotate_y(a) @ np.array([.52, 0, 0])
        self.offset = np.array([0, 0, 9.81*.25/.52/p['kh']])
        self.jacp = np.zeros((3, m.nv)); self.jacr = np.zeros((3, m.nv))

    def forces(self, t, scenario, disable_human=False, wrench_override=None):
        m, d, p = self.m, self.d, self.p
        # Refresh positions/velocities before constructing physical forces.
        mujoco.mj_fwdPosition(m, d); mujoco.mj_fwdVelocity(m, d)
        u, du = smooth(t, .7, p['ramp_duration'])
        a = p['initial_angle']
        da = p['direction']*p['target_angle'] if scenario == 'A' else 0.
        goal_a = a+da*u
        r = rotate_y(goal_a) @ np.array([.52, 0, 0])
        target = PIVOT+r+self.offset
        target_v = clip_norm(np.cross([0, da*du, 0], r), p['human_target_speed_limit'])
        hpoint = d.site_xpos[self.hsite].copy()
        mujoco.mj_jacSite(m, d, self.jacp, self.jacr, self.hsite)
        hvelocity = self.jacp @ d.qvel
        fh = clip_norm(p['kh']*(target-hpoint)+p['dh']*(target_v-hvelocity), p['human_force_limit'])
        if disable_human: fh[:] = 0
        ep = d.xpos[self.panel]+d.xmat[self.panel].reshape(3, 3) @ np.array(p['disturbance_point'])
        fe = np.zeros(3)
        if scenario == 'C':
            on, _ = smooth(t, p['disturbance_start'], .15)
            off, _ = smooth(t, p['disturbance_end'], .15)
            # Downward force creates positive pitch; reverse for negative trials.
            fe[2] = -p['direction']*p['disturbance_magnitude']*(on-off)
        d.qfrc_applied[:] = 0
        if wrench_override is None:
            mujoco.mj_applyFT(m, d, fh, np.zeros(3), hpoint, self.panel, d.qfrc_applied)
            mujoco.mj_applyFT(m, d, fe, np.zeros(3), ep, self.panel, d.qfrc_applied)
        else:
            # Equivalence negative control: identical resultant at the COM.
            f, torque = wrench_override
            mujoco.mj_applyFT(m, d, f, torque, d.xipos[self.panel], self.panel, d.qfrc_applied)
        mujoco.mj_forward(m, d)
        return dict(human_force=fh, external_force=fe, human_point=hpoint,
                    external_point=ep, human_target=target, human_target_velocity=target_v,
                    human_goal_angle=goal_a)

    def observe(self):
        m, d = self.m, self.d
        wr = d.xmat[self.wrist].reshape(3, 3)
        wp = d.xpos[self.wrist]
        grid = np.zeros((2, 3, 4, 4))  # normal magnitude, signed pad-x / pad-y shear [N]
        counts = np.zeros(2, dtype=int)
        normal = np.zeros(2); copnum = np.zeros((2, 2))
        wrench = np.zeros(6); panel_force = np.zeros(3)
        contact_q = np.zeros(m.nv)
        raw = []; slips = []; depths = []
        force = np.zeros(6)
        for ci in range(d.ncon):
            c = d.contact[ci]
            if self.pg not in (c.geom1, c.geom2): continue
            other = c.geom2 if c.geom1 == self.pg else c.geom1
            if other not in self.pad_g: continue
            k = self.pad_g.index(other)
            mujoco.mj_contactForce(m, d, ci, force)
            if force[0] <= 1e-8: continue
            frame = c.frame.reshape(3, 3)
            # Contact frame force acts on geom2. Wrench convention: panel ON gripper.
            world_f = frame.T @ force[:3] * (1 if c.geom2 == other else -1)
            world_t = frame.T @ force[3:] * (1 if c.geom2 == other else -1)
            rot = d.xmat[self.pad_b[k]].reshape(3, 3)
            local_pos = rot.T @ (c.pos-d.xpos[self.pad_b[k]])
            local_f = rot.T @ world_f
            ix, iy = np.clip(np.floor((local_pos[:2]/[.09, .14]+.5)*4).astype(int), 0, 3)
            grid[k, :, iy, ix] += [force[0], local_f[0], local_f[1]]
            counts[k] += 1; normal[k] += force[0]
            copnum[k] += force[0]*local_pos[:2]
            wrench[:3] += wr.T @ world_f
            wrench[3:] += wr.T @ (np.cross(c.pos-wp, world_f)+world_t)
            panel_force -= world_f
            mujoco.mj_applyFT(m, d, -world_f, -world_t, c.pos, self.panel, contact_q)
            jp1 = np.zeros((3, m.nv)); jp2 = np.zeros_like(jp1)
            mujoco.mj_jac(m, d, jp1, None, c.pos, self.panel)
            mujoco.mj_jac(m, d, jp2, None, c.pos, self.pad_b[k])
            rel = (jp1-jp2) @ d.qvel
            slip = np.linalg.norm(rel-np.dot(rel, frame[0])*frame[0])
            slips.append(slip); depths.append(max(0., -c.dist))
            raw.append([k, *local_pos, *c.pos, *local_f, force[0], slip, c.dist])
        cop = np.divide(copnum, normal[:, None], out=np.full((2, 2), np.nan), where=normal[:, None]>0)
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, self.panel, velocity, 0)
        rot = d.xmat[self.panel].reshape(3, 3)
        angle = np.arctan2(-rot[2, 0], rot[0, 0])
        plugin = np.stack([d.sensor(x+'_grid').data.reshape(3, 4, 4).copy() for x in ('upper', 'lower')])
        return dict(wrist_ft=wrench, object_pose=np.r_[d.xpos[self.panel], d.xquat[self.panel]],
                    object_velocity=velocity, angle=angle, tactile_map=plugin,
                    planar_contact_map=grid, contact_count=counts, contact_center=cop,
                    normal_load=normal, slip_speed=max(slips, default=0.),
                    penetration=max(depths, default=0.), panel_contact_force=panel_force,
                    constraint_reconstruction_error=np.max(np.abs(contact_q[self.vadr:self.vadr+6]-d.qfrc_constraint[self.vadr:self.vadr+6])),
                    panel_acceleration=d.qacc[self.vadr:self.vadr+3].copy(),
                    fixture_angle=d.qpos[0], raw_contacts=raw)

    def settle(self):
        for _ in range(round(self.p['settle_seconds']/self.m.opt.timestep)):
            self.forces(-1., 'B'); mujoco.mj_step(self.m, self.d)


def run_episode(p, scenario, out, render=False):
    sim = Pilot(p); sim.settle()
    obs, truth, diag = {}, {}, {}
    rows = []
    dt = sim.m.opt.timestep
    stride = round(1/p['sample_hz']/dt)
    max_accel = 0.; max_pen = 0.; min_contacts = 100; lost_steps = 0; floor_steps = 0
    for step in range(round(p['duration']/dt)):
        t = step*dt
        gt = sim.forces(t, scenario)
        max_accel=max(max_accel,float(np.linalg.norm(sim.d.qacc[sim.vadr:sim.vadr+3])))
        all_pc=[c for c in sim.d.contact if sim.pg in (c.geom1,c.geom2)]
        floor_steps += any(sim.m.geom('floor').id in (c.geom1,c.geom2) for c in all_pc)
        max_pen=max(max_pen,max((-c.dist for c in all_pc),default=0.))
        n=sum((c.geom1 in sim.pad_g or c.geom2 in sim.pad_g) for c in all_pc)
        min_contacts=min(min_contacts,n); lost_steps += n==0
        if step % stride == 0:
            o=sim.observe(); sample=step//stride
            for r in o.pop('raw_contacts'): rows.append([sample,*r])
            residual=sim.m.body_mass[sim.panel]*o['panel_acceleration']-(o['panel_contact_force']+gt['human_force']+gt['external_force']+sim.m.body_mass[sim.panel]*sim.m.opt.gravity)
            o['linear_balance_residual']=np.linalg.norm(residual)
            diag_keys=('penetration','panel_contact_force','constraint_reconstruction_error','panel_acceleration','linear_balance_residual','fixture_angle')
            for k,v in o.items(): (diag if k in diag_keys else obs).setdefault(k,[]).append(v)
            obs.setdefault('time',[]).append(t)
            for k,v in gt.items(): truth.setdefault(k,[]).append(v)
            if render and step==round(1.8/dt):
                render_scene(sim, out/'scene.png')
        mujoco.mj_step(sim.m,sim.d)
        if not np.all(np.isfinite(sim.d.qpos)) or sim.d.time < p['settle_seconds']:
            raise RuntimeError('Nonfinite state or MuJoCo reset')
    out.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(out/'observations.npz',**{k:np.array(v) for k,v in obs.items()})
    np.savez_compressed(out/'privileged.npz',**{k:np.array(v) for k,v in truth.items()},scenario_label=scenario,human_intent='ROTATE' if scenario=='A' else 'HOLD')
    np.savez_compressed(out/'diagnostics.npz',**{k:np.array(v) for k,v in diag.items()},raw_contacts=np.array(rows).reshape(-1,14))
    metadata={**p,'scenario':scenario,'human_intent':'ROTATE' if scenario=='A' else 'HOLD',
        'mujoco_version':mujoco.__version__,'python_version':platform.python_version(),
        'numpy_version':np.__version__,'platform':platform.platform(),
        'model_sha256':hashlib.sha256(MODEL.read_bytes()).hexdigest(),
        'sensor_kind':'reconstructed_contact_wrench','wrench_frame':'wrist body; force panel-on-gripper; torque about wrist origin',
        'tactile_kind':'official touch_grid angular bins, 3x4x4 per pad; plus sparse planar force bins, not pressure',
        'object_velocity_order':'world angular xyz, linear xyz',
        'raw_columns':['sample','pad','local_x','local_y','local_z','world_x','world_y','world_z','pad_fx','pad_fy','pad_fz','normal_N','slip_m_s','distance_m'],
        'validation':{'max_linear_acceleration_m_s2':max_accel,'max_penetration_m':max_pen,
            'minimum_geometric_pad_contacts':min_contacts,'contact_loss_steps':lost_steps,'floor_contact_steps':floor_steps,
            'warning_counts':sim.d.warning.number.tolist(),
            'max_balance_residual_N':max(diag['linear_balance_residual']),
            'max_contact_generalized_force_error':max(diag['constraint_reconstruction_error'])}}
    (out/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    return metadata


def render_scene(sim, path):
    path.parent.mkdir(parents=True,exist_ok=True)
    try:
        import matplotlib
        matplotlib.use('Agg')
        from matplotlib import pyplot as plt
        cam=mujoco.MjvCamera(); cam.lookat[:]=[0,0,.8]; cam.distance=1.35; cam.azimuth=125;cam.elevation=-24
        with mujoco.Renderer(sim.m,480,640) as r:
            r.update_scene(sim.d,camera=cam)
            plt.imsave(path,r.render())
    except Exception as e:
        (path.parent/'render_error.txt').write_text(str(e))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--episodes',type=int,default=10)
    parser.add_argument('--seed',type=int,default=64500)
    parser.add_argument('--output',type=Path,default=ROOT/'results')
    parser.add_argument('--render',action='store_true',help='Optional offscreen images; never required for simulation')
    args=parser.parse_args()
    if args.episodes<1: parser.error('--episodes must be positive')
    args.output.mkdir(parents=True,exist_ok=True)
    manifest=[]
    for i in range(args.episodes):
        for scenario in 'ABC':
            episode=f'{scenario}_{i:03d}'
            dest=args.output/'data'/episode
            if dest.exists(): raise FileExistsError(f'{dest} exists; choose a fresh --output to preserve evidence')
            meta=run_episode(parameters(i,args.seed),scenario,dest,args.render and i==0)
            manifest.append({'episode':episode,'scenario':scenario,'seed':meta['seed'],'model_sha256':meta['model_sha256']})
            print(episode,json.dumps(meta['validation']),flush=True)
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')

if __name__=='__main__': main()
