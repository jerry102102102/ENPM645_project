"""Collect synchronized observations, missing expert references, controls and truth."""
import argparse
import hashlib
import json
import platform
from pathlib import Path
import mujoco
import numpy as np
from phase0_tactile.scripts.run_experiment import render_scene
from phase0_v2.simulation.dual_gripper import DualPilot,parameters,MODEL,ROOT
from phase0_v2.sensing.contacts import observe,gripper_balance
from phase0_v2.control.baseline import MODE,FORCE_LIMIT,TORQUE_LIMIT,KPOS,DPOS,KROT,DROT
from phase0_v2.control.expert_reference import expert_reference
from phase0_v2.data.episode_schema import schema,missing_references,validate_episode


def run_episode(p,scenario,out,render=False):
    if out.exists():raise FileExistsError(out)
    out.mkdir(parents=True)
    sim=DualPilot(p);sim.settle();obs={};gtlog={};ctrl={};diag={};raw=[]
    dt=sim.m.opt.timestep;stride=round(.01/dt)
    health=dict(contact_loss_steps=[0,0],floor_contact_steps=0,max_acceleration_m_s2=0.,max_penetration_m=0.)
    for step in range(round(p['duration']/dt)):
        t=step*dt;truth,control=sim.forces(t,scenario)
        contacts=[c for c in sim.d.contact if sim.pg in (c.geom1,c.geom2)]
        for g in range(2):health['contact_loss_steps'][g]+=not any(c.geom1 in sim.pad_g[g] or c.geom2 in sim.pad_g[g] for c in contacts)
        health['floor_contact_steps']+=any(sim.m.geom('floor').id in (c.geom1,c.geom2) for c in sim.d.contact)
        health['max_penetration_m']=max(health['max_penetration_m'],max((-c.dist for c in contacts),default=0.))
        health['max_acceleration_m_s2']=max(health['max_acceleration_m_s2'],float(np.linalg.norm(sim.d.qacc[sim.vadr:sim.vadr+3])))
        if step%stride==0:
            o,d=observe(sim)
            raw.extend([[step//stride,*row] for row in d.pop('raw_contacts')])
            m=sim.m;R=sim.d.xmat[sim.panel].reshape(3,3);omega=sim.d.qvel[sim.vadr+3:sim.vadr+6]
            I=m.body_inertia[sim.panel]
            inertia_torque=R@(I*sim.d.qacc[sim.vadr+3:sim.vadr+6]+np.cross(omega,I*omega))
            total=o['aggregated_object_wrench']+truth['human_applied_wrench']+truth['external_disturbance_wrench']
            d['gripper_balance_residual']=gripper_balance(sim,o,control['executed_virtual_drive_wrenches'])
            d['linear_balance_residual_N']=float(np.linalg.norm(m.body_mass[sim.panel]*d['panel_acceleration']-total[:3]-m.body_mass[sim.panel]*m.opt.gravity))
            d['angular_balance_residual_Nm']=float(np.linalg.norm(inertia_torque-total[3:]))
            control['realized_object_wrench']=o['aggregated_object_wrench'].copy()
            for target,values in ((obs,o),(gtlog,truth),(ctrl,control),(diag,d)):
                for k,v in values.items():target.setdefault(k,[]).append(v)
            obs.setdefault('timestamp',[]).append(t)
            ref=expert_reference(t)
            assert not ref.available and ref.torque_Nm is None
            if render and abs(t-1.8)<dt/2:render_scene(sim,out/'scene.png')
        mujoco.mj_step(sim.m,sim.d)
        if not np.all(np.isfinite(sim.d.qpos)) or sim.d.time<p['settle_seconds']:raise RuntimeError('nonfinite state or simulation reset')
    obs={k:np.array(v) for k,v in obs.items()};n=len(obs['timestamp']);obs['episode_id']=np.array(out.name)
    ctrl={k:np.array(v) for k,v in ctrl.items()}
    ctrl.update(timestamp=obs['timestamp'],requested_object_wrench=np.full((n,6),np.nan),
                requested_object_wrench_valid=np.zeros(n,bool),allocated_gripper_wrenches=np.full((n,2,6),np.nan),
                allocated_gripper_wrenches_valid=np.zeros(n,bool),controller_mode=np.array(MODE))
    expert=dict(timestamp=obs['timestamp'],**missing_references(n))
    validate_episode(obs,expert,ctrl)
    np.savez_compressed(out/'observations.npz',**obs)
    np.savez_compressed(out/'expert.npz',**expert)
    np.savez_compressed(out/'controller.npz',**ctrl)
    np.savez_compressed(out/'privileged.npz',timestamp=obs['timestamp'],**{k:np.array(v) for k,v in gtlog.items()},scenario_label=scenario,human_intent='ROTATE' if scenario=='A' else 'HOLD',expert_internal_state_available=False)
    np.savez_compressed(out/'diagnostics.npz',timestamp=obs['timestamp'],**{k:np.array(v) for k,v in diag.items()},raw_contacts=np.array(raw).reshape(-1,15))
    for key in ('linear_balance_residual_N','angular_balance_residual_Nm','contact_generalized_error','wrench_transform_error'):
        health['max_'+key]=float(max(diag[key]))
    health['max_gripper_balance_force_N']=float(np.max(np.abs(np.array(diag['gripper_balance_residual'])[:,:,:3])))
    health['max_gripper_balance_torque_Nm']=float(np.max(np.abs(np.array(diag['gripper_balance_residual'])[:,:,3:])))
    health['warning_counts']=sim.d.warning.number.tolist()
    meta={**p,'scenario':scenario,'human_intent':'ROTATE' if scenario=='A' else 'HOLD','mujoco_version':mujoco.__version__,
          'python_version':platform.python_version(),'numpy_version':np.__version__,'platform':platform.platform(),
          'model_sha256':hashlib.sha256(MODEL.read_bytes()).hexdigest(),'validation':health,
          'baseline':dict(kpos=KPOS.tolist(),dpos=DPOS.tolist(),krot=KROT.tolist(),drot=DROT.tolist(),
                          force_norm_limit_N=FORCE_LIMIT,torque_norm_limit_Nm=TORQUE_LIMIT),
          'raw_columns':['sample','gripper','pad','local_x','local_y','local_z','world_x','world_y','world_z','pad_fx','pad_fy','pad_fz','normal_N','slip_m_s','distance_m']}
    (out/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
    return meta


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,default=ROOT/'results/pilot');ap.add_argument('--episodes',type=int,default=10);ap.add_argument('--seed',type=int,default=64600);ap.add_argument('--render',action='store_true')
    a=ap.parse_args()
    if a.episodes<1:ap.error('episodes must be positive')
    if a.output.exists():ap.error('choose a fresh output directory')
    a.output.mkdir(parents=True);manifest=[]
    (a.output/'schema.json').write_text(json.dumps(schema(),indent=2)+'\n')
    for i in range(a.episodes):
        for s in 'ABC':
            name=f'{s}_{i:03d}';meta=run_episode(parameters(i,a.seed),s,a.output/'data'/name,a.render and i==0)
            manifest.append(dict(episode=name,scenario=s,seed=meta['seed'],model_sha256=meta['model_sha256']))
            print(name,json.dumps(meta['validation']),flush=True)
    (a.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')

if __name__=='__main__':main()
