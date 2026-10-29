"""Physics, causal contact controls, replay, units/transforms and dataset contracts."""
import argparse
import hashlib
import json
import tempfile
from pathlib import Path
import mujoco
import numpy as np
from phase0_v2.simulation.dual_gripper import DualPilot,parameters,MODEL,ROOT
from phase0_v2.sensing.contacts import observe
from phase0_v2.scripts.run_pilot import run_episode
from phase0_v2.data.episode_schema import validate_episode,student_view,STUDENT_INPUTS
from phase0_v2.control.baseline import FORCE_LIMIT,TORQUE_LIMIT


def controls():
    p=parameters(0);a=DualPilot(p);b=DualPilot(p);a.settle();b.settle()
    keys=['wrist_ft_left','wrist_ft_right','tactile_left','tactile_right','object_pose','object_velocity']
    worst={k:0. for k in keys}
    for step in range(1500):
        gt,_=a.forces(step*.002,'A')
        b.forces(step*.002,'B',wrench_override=gt['human_applied_wrench'])
        if step%5==0:
            oa,_=observe(a);ob,_=observe(b)
            for k in keys:worst[k]=max(worst[k],float(np.max(np.abs(oa[k]-ob[k]))))
        mujoco.mj_step(a.m,a.d);mujoco.mj_step(b.m,b.d)
    causal={}
    for mode in ('left_removed','right_removed','both_removed','drives_disabled'):
        s=DualPilot(p);s.settle();z0=float(s.d.xpos[s.panel,2])
        removed={'left_removed':[0],'right_removed':[1],'both_removed':[0,1],'drives_disabled':[]}[mode]
        for g in removed:
            s.m.geom_contype[s.pad_g[g]]=0;s.m.geom_conaffinity[s.pad_g[g]]=0
        for step in range(100):
            s.forces(-1,'B',disable_human=True,disable_robot=mode=='drives_disabled');mujoco.mj_step(s.m,s.d)
        s.forces(-1,'B',disable_human=True,disable_robot=mode=='drives_disabled');o,_=observe(s)
        causal[mode]=dict(initial_z=z0,final_z=float(o['object_pose'][2]),
                         contact_counts=[o['contact_state_left'].tolist(),o['contact_state_right'].tolist()],
                         plugin_max_abs=[float(np.max(np.abs(o['tactile_'+side]))) for side in ('left','right')],
                         wrench_norm=[float(np.linalg.norm(o['wrist_ft_'+side])) for side in ('left','right')])
    traces=[]
    for dt in (.002,.001):
        q={**p,'timestep':dt};s=DualPilot(q);s.settle();trace=[]
        for step in range(round(3/dt)):
            s.forces(step*dt,'C')
            if step%round(.01/dt)==0:
                o,_=observe(s);trace.append(np.r_[o['pitch'],o['wrist_ft_left'],o['wrist_ft_right']])
            mujoco.mj_step(s.m,s.d)
        traces.append(np.array(trace))
    rmse=np.sqrt(np.mean((traces[0]-traces[1])**2,axis=0))
    force_ids=[1,2,3,7,8,9];torque_ids=[4,5,6,10,11,12]
    ok=all(v<1e-7 for v in worst.values())
    for mode,g in [('left_removed',0),('right_removed',1)]:
        ok &= causal[mode]['plugin_max_abs'][g]==0 and causal[mode]['wrench_norm'][g]==0 and sum(causal[mode]['contact_counts'][1-g])>0
    ok &= causal['both_removed']['plugin_max_abs']==[0.,0.] and causal['both_removed']['final_z']<causal['both_removed']['initial_z']-.15
    ok &= causal['drives_disabled']['final_z']<causal['drives_disabled']['initial_z']-.15
    ok &= rmse[0]<.01 and max(rmse[force_ids])<.1 and max(rmse[torque_ids])<.02
    return dict(passed=bool(ok),equivalent_resultant_max_abs=worst,causal_controls=causal,
                timestep_halving_rmse=dict(pitch_rad=float(rmse[0]),left_wrist=rmse[1:7].tolist(),right_wrist=rmse[7:].tolist()),
                structure=dict(equality_constraints=int(a.m.neq),free_roots=2,fixed_world_fixture=False,gripper_nominal_separation_m=.4),
                model_sha256=hashlib.sha256(MODEL.read_bytes()).hexdigest())


def validate_dataset(out):
    manifest=json.loads((out/'manifest.json').read_text());maxima={};n=0
    for row in manifest:
        folder=out/'data'/row['episode'];meta=json.loads((folder/'metadata.json').read_text());v=meta['validation']
        assert meta['model_sha256']==hashlib.sha256(MODEL.read_bytes()).hexdigest()
        assert sum(v['contact_loss_steps'])==v['floor_contact_steps']==sum(v['warning_counts'])==0
        assert v['max_penetration_m']<.001 and v['max_acceleration_m_s2']<50
        for k,value in v.items():
            if k.startswith('max_'):maxima[k]=max(maxima.get(k,0),value)
        for k in ('max_linear_balance_residual_N','max_angular_balance_residual_Nm','max_contact_generalized_error','max_wrench_transform_error','max_gripper_balance_force_N','max_gripper_balance_torque_Nm'):
            assert v[k]<1e-8,(row['episode'],k,v[k])
        with np.load(folder/'observations.npz') as o,np.load(folder/'expert.npz') as e,np.load(folder/'controller.npz') as c,np.load(folder/'privileged.npz') as t:
            validate_episode(o,e,c);assert len(o['timestamp'])==400;assert np.allclose(np.diff(o['timestamp']),.01)
            assert set(student_view(o))==set(STUDENT_INPUTS)
            assert all('human' not in k and 'expert' not in k and 'scenario' not in k for k in student_view(o))
            assert np.array_equal(t['timestamp'],o['timestamp'])
            for side in ('left','right'):
                assert np.all(o['contact_state_'+side]>0)
                assert np.allclose(o['planar_tactile_'+side][:,:,0].sum(axis=(2,3)),o['normal_load_'+side])
            assert np.allclose(o['independent_object_wrenches'].sum(axis=1),o['aggregated_object_wrench'])
            assert np.array_equal(c['realized_object_wrench'],o['aggregated_object_wrench'])
            assert np.max(np.linalg.norm(c['executed_virtual_drive_wrenches'][:,:,:3],axis=2))<=FORCE_LIMIT+1e-10
            assert np.max(np.linalg.norm(c['executed_virtual_drive_wrenches'][:,:,3:],axis=2))<=TORQUE_LIMIT+1e-10
            assert np.max(np.abs(c['executed_actuator_commands']))<=80
            assert np.max(np.linalg.norm(t['human_applied_wrench'][:,:3],axis=1))<=50+1e-10
            n+=400
    # Reproduce one complete synchronized episode, including explicit NaN missing fields.
    errors={}
    with tempfile.TemporaryDirectory() as tmp:
        dest=Path(tmp)/'A_000';saved=out/'data/A_000';p=json.loads((saved/'metadata.json').read_text());run_episode(p,'A',dest)
        for filename in ('observations.npz','privileged.npz','diagnostics.npz','expert.npz','controller.npz'):
            with np.load(saved/filename) as a,np.load(dest/filename) as b:
                assert a.files==b.files
                for k in a.files:
                    assert np.array_equal(a[k],b[k],equal_nan=True) if a[k].dtype.kind not in 'US' else np.array_equal(a[k],b[k]),(filename,k)
            errors[filename]='exact equality including missing-value masks'
    return dict(passed=True,episodes=len(manifest),samples=n,maxima=maxima,replay=errors,
                expert_valid_samples=0,student_allowlist_verified=True)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,default=ROOT/'results/pilot');a=ap.parse_args()
    c=controls();(a.output/'physics_controls.json').write_text(json.dumps(c,indent=2)+'\n');print(json.dumps(c,indent=2),flush=True)
    if not c['passed']:raise SystemExit('physical controls failed')
    d=validate_dataset(a.output);(a.output/'dataset_validation.json').write_text(json.dumps(d,indent=2)+'\n');print(json.dumps(d,indent=2))

if __name__=='__main__':main()
