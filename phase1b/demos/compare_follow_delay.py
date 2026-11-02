"""Matched 0/200 ms stop/reverse comparison with standard episodes and local visuals."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from phase1b.benchmark import physical_guard, summarize
from phase1b.hardware import sha
from phase1b.human import HandTargets, HumanWorld
from phase1b.human_follow import DelayedHandReference, KnownInputPolicy, KnownInputObjectReference, PositionFollowArms, matrix
from phase1b.pipeline import run_episode, read_episode, stack
from phase1b.simulation import ROOT

EVENTS=((1.,'ROTATE_POS',30.), (1.8,'HOLD',None), (3.,'ROTATE_POS',30.), (3.8,'ROTATE_NEG',-15.))
WINDOWS={'Stop':(1.8,2.9),'Reverse':(3.8,4.9)}
CONFIG=ROOT/'phase1b/config/dual_panda_base_candidate.json'


def collect(output):
    fixture=HumanWorld(config_path=CONFIG)
    saved=fixture.prepare()
    np.savez_compressed(output/'shared_initial_state.npz',**saved)
    for delay in (0,200):
        world=HumanWorld(config_path=CONFIG); world.restore(saved)
        hand=HandTargets(limit_deg=90.)
        for event in EVENTS: hand.add(*event)
        world.attach(hand,'matched_delay_comparison')
        (output/'events.json').write_text(json.dumps(hand.events,indent=2)+'\n')
        initial,_=world.snapshot(); pose=np.r_[initial['object_position'],initial['object_orientation']]
        follower=DelayedHandReference(hand,delay)
        folder=output/f'delay_{delay}'
        result=run_episode(folder,'B',8.5,world=world,step_guard=physical_guard,history_length=1,
            policy=KnownInputPolicy(follower),object_controller=KnownInputObjectReference(follower,pose),
            dual_arm_controller=PositionFollowArms(world.m,world.qtarget,pose,joint_reserve_deg=5.),
            episode_context=dict(task_id='MATCHED_DELAY_STOP_REVERSE',follow_delay_ms=delay,
                                 known_human_input_baseline=True,formal_demonstration=False,
                                 initial_state_sha256=sha(output/'shared_initial_state.npz')))
        data,_=read_episode(folder); timing=stack(follower.records)
        np.testing.assert_array_equal(timing['decision_timestamp'],data['observations']['timestamp'])
        timing['arm_command_timestamps']=data['controller']['command_timestamp']
        timing['arm_reference_age_s']=timing['arm_command_timestamps']-timing['source_sample_timestamp'][:,None]
        np.savez_compressed(folder/'follower_timing.npz',**timing)
        (folder/'checks.json').write_text(json.dumps(dict(verification=result,clearance_checks=world.clearance_checks,metrics=summarize(folder)),indent=2)+'\n')
        print(f'{delay} ms: physics, margin and clearance checks passed',flush=True)


def stable_time(t,mask,start,end,seconds):
    n=round(seconds*100)
    for i in np.flatnonzero((t>=start-1e-9)&(t<=end-seconds+1e-9)):
        if mask[i:i+n].all(): return float(t[i])
    return None


def analyze(output):
    runs={delay:read_episode(output/f'delay_{delay}')[0] for delay in (0,200)}
    a,b=runs[0],runs[200]
    np.testing.assert_array_equal(a['render_states']['qpos'][0],b['render_states']['qpos'][0])
    np.testing.assert_array_equal(a['render_states']['qvel'][0],b['render_states']['qvel'][0])
    for key in ('human_target_angle','human_target_trajectory','human_target_velocity','human_target_angular_velocity_y','ground_truth_intent'):
        np.testing.assert_array_equal(a['privileged'][key],b['privileged'][key])
    t=a['observations']['timestamp'];np.testing.assert_array_equal(t,b['observations']['timestamp'])
    series={}
    for delay,data in runs.items():
        o=data['observations']; r0=matrix(o['object_orientation'][0])
        rotations=[matrix(q)@r0.T for q in o['object_orientation']]
        theta=np.rad2deg([np.arctan2(r[0,2],r[0,0]) for r in rotations])
        f=data['privileged']['human_applied_force']
        tactile=np.stack([o['tactile_left'][:, :, 0],o['tactile_right'][:, :, 0]],axis=1)
        series[delay]=dict(angle=theta,velocity=np.rad2deg(o['object_angular_velocity'][:,1]),force=f,
                           force_norm=np.linalg.norm(f,axis=1),tactile=tactile,load=tactile.sum(axis=(2,3,4)))
    target=np.rad2deg(a['privileged']['human_target_angle']-a['privileged']['human_target_angle'][0])
    report=dict(matched_initial_state=True,matched_human_targets=True,
        scope='One deterministic matched simulated input episode. Forces are recomputed responses, not matched imposed forces. No human-distribution or assistance-tracking claim.',
        initial_state_sha256=sha(output/'shared_initial_state.npz'),config_sha256=sha(CONFIG),
        sample_hz=100,stop_speed_threshold_deg_s=.5,stop_dwell_s=.2,reverse_speed_threshold_deg_s=-.5,reverse_dwell_s=.1,
        difference_convention='200 ms minus 0 ms, same simulation timestamp',windows={})
    for name,(start,end) in WINDOWS.items():
        idx=np.flatnonzero((t>=start-1e-9)&(t<=end+1e-9));i=idx[0]
        row=dict(start_s=start,end_s=end,runs={})
        for delay,s in series.items():
            hit=stable_time(t,abs(s['velocity'])<=.5,start,end,.2) if name=='Stop' else stable_time(t,s['velocity']<=-.5,start,end,.1)
            row['runs'][str(delay)]=dict(
                response_time_s=hit, response_delay_from_event_s=None if hit is None else hit-start,
                angle_at_event_deg=float(s['angle'][i]),
                continued_positive_motion_deg=float(s['angle'][idx].max()-s['angle'][i]),
                angle_at_window_end_deg=float(s['angle'][idx[-1]]),
                peak_human_force_n=float(s['force_norm'][idx].max()))
        dtheta=series[200]['angle'][idx]-series[0]['angle'][idx]
        row['max_abs_angle_difference_deg']=float(abs(dtheta).max())
        row['max_abs_force_magnitude_difference_n']=float(abs(series[200]['force_norm'][idx]-series[0]['force_norm'][idx]).max())
        delta=series[200]['tactile'][idx]-series[0]['tactile'][idx]
        row['tactile']=dict(max_single_bin_difference_n_per_bin=float(abs(delta).max()),
            peak_l1_map_difference_n_by_side=abs(delta).sum(axis=(2,3,4)).max(axis=0).tolist(),
            max_abs_total_normal_load_difference_n_by_side=abs(delta.sum(axis=(2,3,4))).max(axis=0).tolist())
        snapshot=round((start+.2)*100)
        row['snapshot']=dict(time_s=float(t[snapshot]),after_event_ms=200,
            angle_difference_deg=float(series[200]['angle'][snapshot]-series[0]['angle'][snapshot]),
            force_magnitude_difference_n=float(series[200]['force_norm'][snapshot]-series[0]['force_norm'][snapshot]))
        report['windows'][name]=row
    (output/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
    with (output/'comparison.csv').open('w',newline='') as file:
        writer=csv.writer(file);writer.writerow(['time_s','human_target_deg','angle_0_deg','angle_200_deg','delta_angle_deg','force_0_N','force_200_N','left_normal_0_N','left_normal_200_N','right_normal_0_N','right_normal_200_N'])
        for i,time in enumerate(t):writer.writerow([time,target[i],series[0]['angle'][i],series[200]['angle'][i],series[200]['angle'][i]-series[0]['angle'][i],series[0]['force_norm'][i],series[200]['force_norm'][i],series[0]['load'][i,0],series[200]['load'][i,0],series[0]['load'][i,1],series[200]['load'][i,1]])
    return t,target,series,report


def figures(output,t,target,series):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors={0:'#2277b4',200:'#df7935'}
    fig,axes=plt.subplots(4,2,figsize=(13,11),sharex='col',constrained_layout=True)
    for col,(name,(start,end)) in enumerate(WINDOWS.items()):
        axes[0,col].set_title(f'{name} command at {start:.2f} s')
        axes[0,col].plot(t,target,color='#747d83',ls=':',label='Human target')
        for delay,s in series.items():
            for row,y in enumerate((s['angle'],s['force_norm'],s['load'][:,0],s['load'][:,1])):
                axes[row,col].plot(t,y,color=colors[delay],label=f'{delay} ms')
        for row,label in enumerate(('Panel angle offset (deg)','Human force magnitude (N)','Left: sum of pad normal loads (N)','Right: sum of pad normal loads (N)')):
            ax=axes[row,col];ax.set_ylabel(label,fontsize=9);ax.set_xlim(start-.3,end)
            mask=(t>=start-.3)&(t<=end)
            ys=([target[mask]] if row==0 else [])+[v[mask] for v in ((series[0]['angle'],series[200]['angle']) if row==0 else (series[0]['force_norm'],series[200]['force_norm']) if row==1 else (series[0]['load'][:,row-2],series[200]['load'][:,row-2]))]
            lo,hi=min(y.min() for y in ys),max(y.max() for y in ys);pad=max((hi-lo)*.12,.05);ax.set_ylim(lo-pad,hi+pad)
            ax.axvline(start,color='#333',lw=1,ls='--');ax.axvline(start+.2,color='#df7935',lw=1,ls=':');ax.grid(alpha=.2)
        axes[0,col].legend(fontsize=9);axes[-1,col].set_xlabel('Simulation time (s)')
    fig.suptitle('Identical input and initial state | 0 ms vs 200 ms follower delay',fontsize=15)
    fig.savefig(output/'comparison.png',dpi=160);plt.close(fig)
    normal_max=float(np.ceil(max(s['tactile'].max() for s in series.values())/5)*5)
    diff=series[200]['tactile']-series[0]['tactile']
    diff_max=max(.1,float(np.ceil(abs(diff).max()*10)/10))
    for name,(start,_) in WINDOWS.items():
        i=round((start+.2)*100)
        fig,axes=plt.subplots(3,4,figsize=(12,9),constrained_layout=True)
        maps=[series[0]['tactile'][i],series[200]['tactile'][i],diff[i]]
        for row,mapping in enumerate(maps):
            for col,(side,pad) in enumerate(((0,0),(0,1),(1,0),(1,1))):
                ax=axes[row,col];grid=mapping[side,pad]
                limit=diff_max if row==2 else normal_max
                im=ax.imshow(grid,origin='lower',cmap='RdBu_r' if row==2 else 'magma',vmin=-limit if row==2 else 0,vmax=limit)
                for y in range(4):
                    for x in range(4):
                        rgba=im.cmap(im.norm(grid[y,x]));fg='black' if sum(rgba[:3])>1.8 else 'white'
                        ax.text(x,y,f'{grid[y,x]:+.2f}' if row==2 else f'{grid[y,x]:.2f}',ha='center',va='center',color=fg,fontsize=9)
                ax.set_xticks([]);ax.set_yticks([])
                ax.set_title(f'{("Left","Right")[side]} {("upper","lower")[pad]} | sum {grid.sum():+.2f} N',fontsize=10)
                if col==0:ax.set_ylabel(('0 ms','200 ms','200 - 0 ms')[row],fontsize=12)
            fig.colorbar(im,ax=axes[row,:],shrink=.8,label='N / bin')
        fig.suptitle(f'{name}: t={t[i]:.2f} s (200 ms after command)\nSame scales across both events; sensor-local normal force, not pressure in Pa',fontsize=13)
        fig.savefig(output/f'tactile_{name.lower()}.png',dpi=160);plt.close(fig)
    return normal_max,diff_max


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--render-only',action='store_true');args=parser.parse_args()
    if not args.render_only:
        args.output.mkdir(parents=True,exist_ok=False);collect(args.output)
    t,target,series,report=analyze(args.output)
    normal_max,diff_max=figures(args.output,t,target,series)
    payload=dict(t=np.round(t,2).tolist(),target=np.round(target,5).tolist(),normal_max=normal_max,diff_max=diff_max,
        events=[dict(time=t,mode=m) for t,m,_ in EVENTS],report=report,
        runs={str(d):{k:np.round(v,5).tolist() for k,v in s.items()} for d,s in series.items()})
    template=Path(__file__).with_name('follow_delay_view.html').read_text()
    (args.output/'index.html').write_text(template.replace('__DATA__',json.dumps(payload,separators=(',',':'))))
    print(json.dumps(report,indent=2),flush=True)

if __name__=='__main__':main()
