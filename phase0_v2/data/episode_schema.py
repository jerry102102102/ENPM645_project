"""A strict student allowlist plus explicit missing supervision/controller fields."""
import numpy as np
from phase0_v2.control.interface import ASSIST_CONVENTION
from phase0_v2.control.expert_reference import EXPERT_INPUT_CONTRACT

# Identity, time and calibration are synchronizing metadata, never scenario features.
STUDENT_INPUTS = (
    'wrist_ft_left','wrist_ft_right','tactile_left','tactile_right',
    'object_pose','object_velocity','contact_state_left','contact_state_right',
)


def student_view(observations):
    return {key:observations[key] for key in STUDENT_INPUTS}


def schema():
    return {
      'schema_version':'enpm645-force-demo/2.0', 'student_input_allowlist':list(STUDENT_INPUTS),
      'identity_fields_not_model_inputs':['timestamp','episode_id'],
      'wrist_ft':{'kind':'reconstructed_contact_wrench','order':'Fx Fy Fz Tx Ty Tz',
                  'frame':'each gripper root body','origin':'each root origin','direction':'panel on gripper'},
      'object_wrench':{'frame':'world','origin':'instantaneous panel COM','direction':'grippers on panel','order':'Fx Fy Fz Tx Ty Tz'},
      'object_pose':'world xyz + quaternion wxyz', 'object_velocity':'world angular xyz + linear xyz',
      'tactile':{'shape_per_hand':[2,3,4,4],'pads':['upper','lower'],'channels':'sensor z x y',
                 'units':'N per angular bin, not pressure','plugin':'mujoco.sensor.touch_grid'},
      'planar_tactile':'2 pads x (normal magnitude, pad x shear, pad y shear) x 4 x 4; N/bin; no interpolation',
      'contact_state':'2 force-bearing contact counts, one per pad',
      'expert_reference':{**ASSIST_CONVENTION,'availability':False,'missing_encoding':'NaN plus false validity mask',
                           'reason':'No validated object-wrench allocator/tracker'},
      'expert_design':EXPERT_INPUT_CONTRACT,
      'controller':{'requested_object_wrench':'unavailable', 'allocated_gripper_wrenches':'unavailable',
        'executed_actuator_commands':'actual four pad motor forces, N',
        'executed_virtual_drive_wrenches':'actual bounded external drives at root origins, world, N / Nm',
        'realized_object_wrench':'measured contact sum acting on panel, world about COM',
        'fixed_fixture_reaction':'not applicable: each root is freejoint, no world fixture constraint'},
      'calibration':{'force_bias':'none, ideal simulation','sensor_noise':'none','tactile_fov_deg':[110,85],
                      'site_backoff_m':.08,'gamma':0.,'planar_pad_size_m':[.09,.14]},
      'observation_augmentation':'derived wrench/state/contact fields only; no privileged goal/force/label',
      'privileged_files':['privileged.npz','metadata.json'],
    }


def missing_references(n):
    return dict(expert_assist_torque_ref=np.full(n,np.nan),expert_assist_torque_ref_valid=np.zeros(n,bool),
                total_target_torque_ref=np.full(n,np.nan),total_target_torque_ref_valid=np.zeros(n,bool))


def validate_episode(obs, expert, controller):
    n=len(obs['timestamp'])
    assert np.all(np.diff(obs['timestamp'])>0)
    for k in STUDENT_INPUTS:
        assert len(obs[k])==n and np.all(np.isfinite(obs[k])), k
    assert obs['wrist_ft_left'].shape==(n,6)
    assert obs['tactile_left'].shape==(n,2,3,4,4)
    assert not expert['expert_assist_torque_ref_valid'].any()
    assert np.isnan(expert['expert_assist_torque_ref']).all()
    assert np.isnan(controller['requested_object_wrench']).all()
    assert np.isnan(controller['allocated_gripper_wrenches']).all()
    assert np.array_equal(controller['timestamp'],obs['timestamp'])
    assert np.array_equal(expert['timestamp'],obs['timestamp'])
    forbidden={'human_goal','human_target_trajectory','human_applied_wrench','external_disturbance_wrench','scenario_label','expert_internal_state'}
    assert not forbidden.intersection(obs)
