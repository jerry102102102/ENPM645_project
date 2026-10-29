"""Physical and sensor controls independent of scenario classification."""
import argparse
import hashlib
import json
import tempfile
from pathlib import Path
import mujoco
import numpy as np
from run_experiment import Pilot, parameters, run_episode, MODEL, ROOT


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--output',type=Path,default=ROOT/'results/pilot')
    args=ap.parse_args(); args.output.mkdir(parents=True,exist_ok=True)
    p=parameters(0,64500)
    a=Pilot(p); b=Pilot(p); a.settle(); b.settle()
    worst={k:0. for k in ('wrist_ft','object_pose','object_velocity','tactile_map','planar_contact_map')}
    for step in range(1500):
        gt=a.forces(step*.002,'A')
        com=a.d.xipos[a.panel].copy()
        torque=np.cross(gt['human_point']-com,gt['human_force'])
        b.forces(step*.002,'B',wrench_override=(gt['human_force'],torque))
        if step%5==0:
            oa,ob=a.observe(),b.observe()
            for key in worst: worst[key]=max(worst[key],float(np.max(np.abs(oa[key]-ob[key]))))
        mujoco.mj_step(a.m,a.d);mujoco.mj_step(b.m,b.d)
    # Removing all panel/pad contact pairs must remove all tactile signals.
    c=Pilot(p)
    c.m.geom_contype[c.pad_g]=0;c.m.geom_conaffinity[c.pad_g]=0
    for step in range(100):
        c.forces(-1,'B',disable_human=True);mujoco.mj_step(c.m,c.d)
    c.forces(-1,'B',disable_human=True)
    oc=c.observe()
    no_contact={'pad_contacts':int(oc['contact_count'].sum()),'plugin_max_N':float(np.max(np.abs(oc['tactile_map']))),'map_max_N':float(np.max(np.abs(oc['planar_contact_map']))),'panel_z_m':float(oc['object_pose'][2])}
    # Timestep sensitivity, not a change to the main dataset.
    traces=[]
    for dt in (.002,.001):
        s=Pilot(p);s.m.opt.timestep=dt;s.settle();samples=[]
        for step in range(round(3/dt)):
            s.forces(step*dt,'C')
            if step%round(.01/dt)==0:
                o=s.observe();samples.append(np.r_[o['angle'],o['wrist_ft']])
            mujoco.mj_step(s.m,s.d)
        traces.append(np.array(samples))
    err=np.sqrt(np.mean((traces[0]-traces[1])**2,axis=0))
    checks={'equivalent_resultant_control_max_abs':worst,'no_contact_control':no_contact,
            'timestep_halving_rmse':{'angle_rad':float(err[0]),'force_N':err[1:4].tolist(),'torque_Nm':err[4:].tolist()},
            'plugin_available':True,'mujoco_version':mujoco.__version__,
            'model_sha256':hashlib.sha256(MODEL.read_bytes()).hexdigest()}
    # Replay an entire saved episode, compare decoded arrays, not ZIP timestamps.
    saved = args.output/'data/A_000'
    dataset = {'checked': False}
    replay = {'checked': False}
    if saved.exists():
        records = json.loads((args.output/'manifest.json').read_text())
        sample_count = 0
        for record in records:
            folder = args.output/'data'/record['episode']
            meta = json.loads((folder/'metadata.json').read_text())
            assert meta['model_sha256'] == checks['model_sha256']
            assert record['model_sha256'] == checks['model_sha256']
            v = meta['validation']
            assert v['contact_loss_steps'] == v['floor_contact_steps'] == sum(v['warning_counts']) == 0
            assert v['max_penetration_m'] < .001 and v['max_linear_acceleration_m_s2'] < 50
            assert v['max_balance_residual_N'] < 1e-8 and v['max_contact_generalized_force_error'] < 1e-8
            with np.load(folder/'observations.npz') as o:
                assert len(o['time']) == 400 and np.allclose(np.diff(o['time']), .01)
                assert all(np.all(np.isfinite(o[k])) for k in o.files)
                assert np.all(o['contact_count'] > 0)
                assert np.allclose(o['planar_contact_map'][:, :, 0].sum(axis=(2,3)), o['normal_load'])
                sample_count += len(o['time'])
            with np.load(folder/'privileged.npz') as truth:
                assert np.max(np.linalg.norm(truth['human_force'],axis=1)) <= meta['human_force_limit']+1e-10
                assert np.max(np.linalg.norm(truth['human_target_velocity'],axis=1)) <= meta['human_target_speed_limit']+1e-10
        dataset = {'checked': True, 'episodes': len(records), 'samples': sample_count,
                   'finite_signals_force_limits_model_binding_contact_balance': True}
        meta = json.loads((saved/'metadata.json').read_text())
        replay_error = {}
        with tempfile.TemporaryDirectory() as tmp:
            run_episode(meta, 'A', Path(tmp))
            for filename in ('observations.npz','privileged.npz','diagnostics.npz'):
                with np.load(saved/filename) as original, np.load(Path(tmp)/filename) as repeated:
                    assert original.files == repeated.files
                    maximum = 0.
                    for key in original.files:
                        if original[key].dtype.kind in 'US':
                            assert np.array_equal(original[key], repeated[key])
                        else:
                            maximum = max(maximum, float(np.max(np.abs(original[key]-repeated[key]))))
                    replay_error[filename] = maximum
        replay = {'checked': True, 'episode': 'A_000', 'max_abs_difference': replay_error}
        assert all(v < 1e-10 for v in replay_error.values())
    checks['dataset_validation'] = dataset
    checks['deterministic_replay'] = replay
    checks['passed']=bool(all(v<1e-5 for v in worst.values()) and no_contact['pad_contacts']==0 and no_contact['plugin_max_N']==0 and no_contact['map_max_N']==0 and no_contact['panel_z_m']<.7 and err[0]<.01 and max(err[1:4])<.05 and max(err[4:])<.01)
    (args.output/'physics_controls.json').write_text(json.dumps(checks,indent=2)+'\n')
    print(json.dumps(checks,indent=2))
    if not checks['passed']: raise SystemExit('Physics controls failed; inspect results')

if __name__=='__main__':main()
