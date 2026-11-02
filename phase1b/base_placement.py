"""Offline base-placement study; never changes the active hardware or human runner.

Candidate XMLs and simulation evidence live in a fresh output directory. Keep the
panel, human point, gripper transforms, actuator parameters and joint limits fixed.
"""
import argparse
import copy
import itertools
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from phase1b.geometry import rotate_y, solve
from phase1b.hardware import sha
from phase1b.human_follow import matrix
from phase1b.simulation import PandaWorld, ROOT, CONFIG


def place(model, placement):
    """Change only fixed base transforms and their matching pedestal cylinders."""
    x, y, z, toe = placement
    for sign, side in ((-1, 'left'), (1, 'right')):
        yaw = np.deg2rad(-90 - sign*toe)
        model.body(side+'_link0').pos[:] = [sign*x, y, z]
        model.body(side+'_link0').quat[:] = [np.cos(yaw/2), 0, 0, np.sin(yaw/2)]
        model.geom(side+'_pedestal').pos[:] = [sign*x, y, z/2]
        model.geom(side+'_pedestal').size[1] = z/2


def unexpected_contacts(world, model, data):
    pads = sum(world.pad_g, [])
    return [dict(body1=model.body(model.geom_bodyid[c.geom1]).name,
                 body2=model.body(model.geom_bodyid[c.geom2]).name, depth_m=float(-c.dist))
            for c in data.contact if c.dist < -1e-5
            and not (world.pg in (c.geom1, c.geom2) and (c.geom1 in pads or c.geom2 in pads))]


def margin(world, model, data):
    q = data.qpos[world.qa]
    bounds = world.m.jnt_range[world.arm_joints]
    return float(np.rad2deg(np.minimum(q-bounds[...,0], bounds[...,1]-q).min()))


def sample_path(world, model, qpos, positions, rotations, extent, step):
    rows = []
    pivot = world.d.xipos[world.panel].copy()
    panel0 = world.d.xmat[world.panel].reshape(3,3).copy()
    pa = model.jnt_qposadr[model.joint('panel_free').id]
    for direction in (1,-1):
        d = mujoco.MjData(model)
        d.qpos[:] = qpos
        for angle in np.arange(0, extent+1e-8, step):
            delta = rotate_y(np.deg2rad(direction*angle))
            good = True
            for arm, side in enumerate(('left','right')):
                result = solve(model, d, side, pivot+delta@(positions[arm]-pivot),
                               delta@rotations[arm], d.qpos[world.qa[arm]].copy())
                if result is None:
                    good = False
                    break
            if not good:
                rows.append(dict(angle_deg=float(direction*angle), passed=False, reason='ik_failed'))
                break
            R = delta@panel0
            quat = np.zeros(4)
            mujoco.mju_mat2Quat(quat, R.ravel())
            d.qpos[pa:pa+3] = pivot-R@model.body_ipos[world.panel]
            d.qpos[pa+3:pa+7] = quat
            mujoco.mj_forward(model,d)
            collisions = unexpected_contacts(world,model,d)
            rows.append(dict(angle_deg=float(direction*angle), passed=not bool(collisions),
                             min_joint_margin_deg=margin(world,model,d), collisions=collisions,
                             qpos=d.qpos.tolist()))
            if collisions:
                break
    return rows


def export_model(output, world, placement, qpos):
    tree = ET.parse(ROOT/world.config['model'])
    root = tree.getroot()
    root.set('model','dual_panda_base_placement_candidate')
    root.find('compiler').set('meshdir',str(ROOT/'phase1b/vendor/franka_emika_panda/assets'))
    x,y,z,toe = placement
    for sign,side in ((-1,'left'),(1,'right')):
        yaw=np.deg2rad(-90-sign*toe)
        root.find(f".//body[@name='{side}_link0']").set('pos',f'{sign*x} {y} {z}')
        root.find(f".//body[@name='{side}_link0']").set('quat',f'{np.cos(yaw/2)} 0 0 {np.sin(yaw/2)}')
        p=root.find(f".//geom[@name='{side}_pedestal']")
        p.set('pos',f'{sign*x} {y} {z/2}')
        p.set('size',f'.11 {z/2}')
    keyframe=root.find('keyframe')
    if keyframe is not None:
        root.remove(keyframe)
    keyframe=ET.SubElement(root,'keyframe')
    ET.SubElement(keyframe,'key',name='candidate_nominal',qpos=' '.join(map(str,qpos)))
    tree.write(output, encoding='unicode')


def study(output, extent=100., step=10., reserve=0., wide=False):
    output=Path(output); output.mkdir(parents=True,exist_ok=False)
    world=PandaWorld()
    sites=[world.m.site(s+'_ee').id for s in ('left','right')]
    positions=world.d.site_xpos[sites].copy()
    rotations=world.d.site_xmat[sites].reshape(2,3,3).copy()
    configurations=[(.45,.65,.25,0.)]+list(itertools.product((.35,.5),(.45,.6,.75),(.2,.4),(0.,30.)))
    if wide:
        configurations=[(.45,.65,.25,0.)]+list(itertools.product((.25,.4,.55),(.3,.45,.6),(.05,.2,.4,.6),(0.,30.)))
    results=[]
    for number, placement in enumerate(configurations):
        model=copy.copy(world.m); place(model,placement)
        model.jnt_range[world.arm_joints,0] += np.deg2rad(reserve)
        model.jnt_range[world.arm_joints,1] -= np.deg2rad(reserve)
        d=mujoco.MjData(model); d.qpos[:]=world.d.qpos
        nominal=[]
        for arm,side in enumerate(('left','right')):
            seeds=[world.qtarget[arm], np.array([0,-.4,0,-1.8,0,1.8,.7]),
                   np.array([0,.4,0,-2.2,0,2.,-.7])]
            rng=np.random.default_rng(645+arm)
            seeds += [np.array([rng.uniform(-1,1),rng.uniform(-.8,.8),rng.uniform(-1.5,1.5),
                               -rng.uniform(1.3,2.5),rng.uniform(-1.5,1.5),rng.uniform(1.2,2.8),rng.uniform(-1.5,1.5)]) for _ in range(3)]
            candidates=[]
            for seed in seeds:
                solved=solve(model,d,side,positions[arm],rotations[arm],seed)
                if solved:
                    q=d.qpos[world.qa[arm]].copy(); bounds=model.jnt_range[world.arm_joints[arm]]
                    candidates.append((np.minimum(q-bounds[:,0],bounds[:,1]-q).min(),q))
            if not candidates:
                break
            nominal.append(max(candidates,key=lambda x:x[0])[1])
        row=dict(id=number, placement=list(placement), full_path_passed=False)
        if len(nominal)==2:
            d.qpos[:]=world.d.qpos; d.qpos[world.qa]=nominal
            mujoco.mj_forward(model,d)
            row['nominal_qpos']=d.qpos.tolist()
            row['samples']=sample_path(world,model,d.qpos.copy(),positions,rotations,extent,step)
            row['full_path_passed']=all(r['passed'] for r in row['samples'])
            row['min_joint_margin_deg']=min((r.get('min_joint_margin_deg',0) for r in row['samples']),default=0)
            row['symmetric_passed_extent_deg']=min(max([abs(r['angle_deg']) for r in row['samples'] if r['passed'] and r['angle_deg']*sign>=0],default=0) for sign in (1,-1))
        else:
            row['reason']='nominal_ik_failed'
        results.append(row)
        (output/'search.json').write_text(json.dumps(dict(extent_deg=extent,step_deg=step,
            scope='Sampled local IK branches, enabled MJCF collision pairs; no dynamics or global optimality.',
            planning_reserve_deg=reserve,
            original_model_sha256=world.config['model_sha256'], source_sha256=sha(__file__),
            placement_columns=['half_spacing_x_m','base_y_m','base_z_m','inward_yaw_deg'],candidates=results),indent=2)+'\n')
        print(number, placement, row['full_path_passed'], round(row.get('min_joint_margin_deg',0),2),row.get('symmetric_passed_extent_deg',0),flush=True)
    ranked=sorted((r for r in results if r['full_path_passed']),key=lambda r:r['min_joint_margin_deg'],reverse=True)
    for r in ranked[:3]:
        export_model(output/f"candidate_{r['id']}.xml",world,r['placement'],r['nominal_qpos'])
    return ranked


def validate_candidate(output, search_result, candidate_id, extent=90., step=2., reserve=5.):
    """Independent candidate capture, with actual model/hash and no formal labels."""
    from phase1b.human import HumanWorld, HandTargets
    from phase1b.human_follow import KnownInputPolicy, KnownInputObjectReference, PositionFollowArms
    from phase1b.control import ControlChain
    from phase1b.pipeline import controller_input, stack
    from phase1b.benchmark import physical_guard
    from phase1b.hardware import snapshot

    output=Path(output); output.mkdir(parents=True,exist_ok=False)
    search=json.loads(Path(search_result).read_text())
    candidate=next(c for c in search['candidates'] if c['id']==candidate_id)
    world=HumanWorld()
    old=world.m
    export_model(output/'scene.xml',world,candidate['placement'],candidate['nominal_qpos'])
    m=mujoco.MjModel.from_xml_path(str(output/'scene.xml'))
    m.geom_friction[:,0]=world.p['friction']
    # Only the two base transforms and two matching pedestal transforms/sizes may differ.
    for name in snapshot(world)['arrays']:
        before, after=getattr(old,name),getattr(m,name)
        if name in ('body_pos','body_quat'):
            ids=[m.body(s+'_link0').id for s in ('left','right')]
        elif name in ('geom_pos','geom_size'):
            ids=[m.geom(s+'_pedestal').id for s in ('left','right')]
        else:
            ids=[]
        mask=np.ones(len(before),dtype=bool); mask[ids]=False
        np.testing.assert_array_equal(before[mask],after[mask],err_msg=name)
    world.m=m; world.d=mujoco.MjData(m)
    world.d.qpos[:]=candidate['nominal_qpos']
    world.qtarget=world.d.qpos[world.qa].copy()
    world.set_commands(world.qtarget,[-40.,40.,-40.,40.])
    mujoco.mj_forward(m,world.d)
    world.config=dict(world.config,model=str(output/'scene.xml'),model_sha256=sha(output/'scene.xml'),
                      scenario_version='3.0.0+base-study',hardware_version='2.0.0+base-study')
    hardware=snapshot(world)
    hardware.update(hardware_version=world.config['hardware_version'],scenario_version=world.config['scenario_version'])
    (output/'hardware.json').write_text(json.dumps(hardware,indent=2)+'\n')
    report=dict(candidate_id=candidate_id,placement=candidate['placement'],passed=False,
                scope='Candidate-only simulation, original following controller, frozen grasps and actuators. Not active hardware or torque-tracking validation.',
                model_sha256=sha(output/'scene.xml'),hardware_sha256=sha(output/'hardware.json'),
                search_sha256=sha(search_result),source_sha256=sha(__file__),
                source_hashes={p.name:sha(p) for p in (ROOT/'phase1b').glob('*.py')},
                timestep_s=.002,physics_samples=0, min_joint_margin_deg=180.,
                min_contacts_per_pad=[999]*4,max_penetration_m=0.,max_human_force_n=0.,
                requested_extent_deg=extent, speed_deg_s=15.,accel_deg_s2=60., planning_joint_reserve_deg=reserve,
                expert_labels_valid=False,assistance_tracking_valid=False)
    rows=[]
    try:
        world.restore(world.prepare())
        np.savez_compressed(output/'initial_state.npz',**world.capture())
        initial,_=world.snapshot(); pose=np.r_[initial['object_position'],initial['object_orientation']]
        sites=[m.site(s+'_ee').id for s in ('left','right')]
        # Refine kinematics from the settled command FK used by the actual baseline.
        planner=PositionFollowArms(m,world.qtarget,pose)
        planner.m.jnt_range[world.arm_joints,0]+=np.deg2rad(reserve)
        planner.m.jnt_range[world.arm_joints,1]-=np.deg2rad(reserve)
        plan_model=copy.copy(m)
        plan_model.jnt_range[world.arm_joints,0]+=np.deg2rad(reserve)
        plan_model.jnt_range[world.arm_joints,1]-=np.deg2rad(reserve)
        q=world.d.qpos.copy(); q[world.qa]=world.qtarget
        path=sample_path(world,plan_model,q,planner.positions,planner.rotations,extent,step)
        (output/'refined_path.json').write_text(json.dumps(path,indent=2)+'\n')
        report['refined_geometry_passed']=all(r['passed'] for r in path)
        report['refined_geometry_min_margin_deg']=min(r.get('min_joint_margin_deg',0.) for r in path)
        hand=HandTargets(limit_deg=extent)
        reverse=float(np.ceil(1+extent/15+.25+1))
        resume=float(np.ceil(reverse+2*extent/15+.25+1))
        events=[(1.,'ROTATE_POS',extent),(reverse,'ROTATE_NEG',-extent),
                (resume,'ROTATE_POS',extent),(resume+2,'HOLD',None),
                (resume+4,'ROTATE_NEG',-extent),(resume+4.5,'ROTATE_POS',0.)]
        for e in events: hand.add(*e)
        (output/'events.json').write_text(json.dumps(hand.events,indent=2)+'\n')
        world.attach(hand,'base_placement_validation')
        chain=ControlChain(policy=KnownInputPolicy(hand),object_controller=KnownInputObjectReference(hand,pose),
                           dual_arm_controller=planner,history_length=1)
        duration=float(np.ceil(resume+4.5+extent/15+2))
        for _ in range(round(duration/.002)):
            o,_=world.snapshot(); command=chain.step(controller_input(world,o))
            world.set_commands(command.arm_position_target,command.pad_force_command)
            world.refresh(world.tick*.002); world.step(); physical_guard(world)
            o,d=world.snapshot()
            report['physics_samples']+=1
            report['min_joint_margin_deg']=min(report['min_joint_margin_deg'],margin(world,m,world.d))
            contacts=np.r_[o['contact_state_left'],o['contact_state_right']]
            report['min_contacts_per_pad']=np.minimum(report['min_contacts_per_pad'],contacts).astype(int).tolist()
            report['max_penetration_m']=max(report['max_penetration_m'],float(d['penetration']))
            report['max_human_force_n']=max(report['max_human_force_n'],float(np.linalg.norm(world.truth['human_applied_force'])))
            if world.tick%5==0:
                R=world.d.xmat[world.panel].reshape(3,3)@matrix(pose[3:]).T
                rows.append(dict(timestamp=world.tick*.002, target_deg=hand.sample(world.tick*.002)[0],
                    measured_deg=float(np.rad2deg(np.arctan2(R[0,2],R[0,0]))),
                    qpos=world.d.qpos.copy(),qvel=world.d.qvel.copy(),command=command.arm_position_target.copy(),
                    min_joint_margin_deg=margin(world,m,world.d),contacts=contacts.copy(),
                    tactile_left=o['tactile_left'].copy(),tactile_right=o['tactile_right'].copy(),
                    human_force=world.truth['human_applied_force'].copy()))
        checkpoints=[]
        for t in (reverse-.1,resume-.1,resume+3.9,duration):
            row=min(rows,key=lambda r:abs(r['timestamp']-t))
            checkpoints.append({k:row[k] for k in ('timestamp','target_deg','measured_deg')})
        report.update(completed=True, checkpoints=checkpoints,
                      checkpoint_tracking_passed=all(abs(r['target_deg']-r['measured_deg'])<.5 for r in checkpoints))
        report['passed']=bool(report['refined_geometry_passed'] and report['checkpoint_tracking_passed'] and report['min_joint_margin_deg']>=5.)
    except (RuntimeError,ValueError) as error:
        report.update(error=str(error),failed_at_seconds=world.tick*.002)
    finally:
        if rows: np.savez_compressed(output/'trace.npz',**stack(rows))
        np.savez_compressed(output/'final_state.npz',qpos=world.d.qpos,qvel=world.d.qvel)
        (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--extent-deg',type=float,default=100.)
    parser.add_argument('--step-deg',type=float,default=10.)
    parser.add_argument('--reserve-deg',type=float,default=0.)
    parser.add_argument('--wide',action='store_true')
    parser.add_argument('--search-result',type=Path)
    parser.add_argument('--candidate-id',type=int)
    args=parser.parse_args()
    if not 0 < args.step_deg <= args.extent_deg <=180:
        parser.error('require 0 < step <= extent <=180')
    if args.search_result is not None:
        validate_candidate(args.output,args.search_result,args.candidate_id,args.extent_deg,args.step_deg,args.reserve_deg)
        return
    ranked=study(args.output,args.extent_deg,args.step_deg,args.reserve_deg,args.wide)
    print('ranked:',[(r['id'],r['placement'],r['min_joint_margin_deg']) for r in ranked[:5]])

if __name__=='__main__':
    main()
