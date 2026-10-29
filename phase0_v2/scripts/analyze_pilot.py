"""S1/S2/S3 comparisons with same-block, full-history matching; no classifier."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from phase0_v2.simulation.dual_gripper import ROOT

W_LABELS=['Fx','Fy','Fz','Tx','Ty','Tz']
STATE_LABELS=['px','py','pz','rotvec_x','rotvec_y','rotvec_z','omega_x','omega_y','omega_z','vx','vy','vz']
WTOL=np.array([.5]*3+[.05]*3)
STOL=np.array([.003]*3+[np.deg2rad(1)]*3+[.03]*3+[.02]*3)
WINDOW=15
STARTS=np.arange(85,236,5)


def save_csv(path,rows):
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows)


def state(o):
    q=o['object_pose'][:,3:].copy();q[q[:,0]<0]*=-1
    n=np.linalg.norm(q[:,1:],axis=1);angle=2*np.arctan2(n,q[:,0])
    r=q[:,1:]*np.divide(angle,n,out=np.full_like(n,2.),where=n>1e-12)[:,None]
    return np.column_stack([o['object_pose'][:,:3],r,o['object_velocity']])


def features(o,level):
    wrench=o['aggregated_object_wrench'] if level=='S1' else o['independent_object_wrenches'].reshape(len(o['timestamp']),12)
    return np.column_stack([wrench,state(o)])


def feature_labels(level):
    return (W_LABELS if level=='S1' else [s+'_'+k for s in ('L','R') for k in W_LABELS])+STATE_LABELS


def tolerance(level):
    return np.r_[WTOL if level=='S1' else np.tile(WTOL,2),STOL]


def tactile_difference(a,c,sa,sc):
    ma=np.stack([a['planar_tactile_'+s][sa] for s in ('left','right')],axis=1)
    mc=np.stack([c['planar_tactile_'+s][sc] for s in ('left','right')],axis=1)
    na, nc=ma[:,:,:,0],mc[:,:,:,0]
    norm_a=na/na.sum(axis=(-2,-1),keepdims=True);norm_c=nc/nc.sum(axis=(-2,-1),keepdims=True)
    copa=np.stack([a['contact_center_'+s][sa] for s in ('left','right')]);copc=np.stack([c['contact_center_'+s][sc] for s in ('left','right')])
    return dict(normal_rmse_N=float(np.sqrt(np.mean((na-nc)**2))),shear_rmse_N=float(np.sqrt(np.mean((ma[:,:,:,1:]-mc[:,:,:,1:])**2))),
                normalized_normal_L1=float(np.mean(np.sum(np.abs(norm_a-norm_c),axis=(-2,-1)))),
                cop_coordinate_rmse_mm=float(1000*np.sqrt(np.mean((copa-copc)**2))))


def match(data):
    rows=[];details=[]
    for name,a in data.items():
        if name[0]!='A':continue
        cname=name.replace('A_','C_');c=data[cname];errors={};scores={};valid={}
        for level in ('S1','S2'):
            aw=np.array([features(a,level)[s:s+WINDOW] for s in STARTS]);cw=np.array([features(c,level)[s:s+WINDOW] for s in STARTS])
            err=np.sqrt(np.mean((aw[:,None]-cw[None,:])**2,axis=2));errors[level]=err
            normalized=err/tolerance(level);scores[level]=np.sqrt(np.mean(normalized**2,axis=2));valid[level]=np.all(normalized<=1,axis=2)
        valid['S2'] &= valid['S1']  # Stronger matching must also preserve the S1 conditions.
        for level in ('S1','S2'):
            objective=np.where(valid[level],scores[level],np.inf) if valid[level].any() else scores[level]
            ix,iy=np.unravel_index(np.argmin(objective),objective.shape);ai,ci=int(STARTS[ix]),int(STARTS[iy]);sa,sc=slice(ai,ai+WINDOW),slice(ci,ci+WINDOW)
            e1,e2=errors['S1'][ix,iy],errors['S2'][ix,iy]
            row=dict(A=name,C=cname,matching_level=level,A_start_s=ai/100,C_start_s=ci/100,
                     score=float(scores[level][ix,iy]),qualified=bool(valid[level][ix,iy]),qualified_candidate_pairs=int(valid[level].sum()),
                     S1_max_tolerance_ratio=float(np.max(e1/tolerance('S1'))),S2_max_tolerance_ratio=float(np.max(e2/tolerance('S2'))),
                     independent_force_rmse_N=float(np.sqrt(np.mean(e2[[0,1,2,6,7,8]]**2))),
                     independent_torque_rmse_Nm=float(np.sqrt(np.mean(e2[[3,4,5,9,10,11]]**2))),
                     gripper_pose_xyz_rmse_mm=float(1000*np.sqrt(np.mean((a['gripper_pose'][sa,:,:3]-c['gripper_pose'][sc,:,:3])**2))),
                     **tactile_difference(a,c,sa,sc))
            for factor in (.5,.25):
                condition=np.all(errors['S1']/tolerance('S1')<=factor,axis=2)
                if level=='S2':condition &= np.all(errors['S2']/tolerance('S2')<=factor,axis=2)
                row[f'qualified_at_{factor}']=int(condition.sum())
            rows.append(row)
            for lev,err in [('S1',e1),('S2',e2)]:
                for label,value,tol in zip(feature_labels(lev),err,tolerance(lev)):
                    details.append(dict(A=name,C=cname,selected_for=level,evaluated_level=lev,component=label,rmse=float(value),tolerance=float(tol),within_tolerance=bool(value<=tol)))
    return rows,details


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,default=ROOT/'results/pilot');a=ap.parse_args();out=a.output
    figs=out/'figures';figs.mkdir(exist_ok=True);data={};truth={};ctrl={};rows=[]
    for item in json.loads((out/'manifest.json').read_text()):
        name=item['episode'];folder=out/'data'/name;o=dict(np.load(folder/'observations.npz'));data[name]=o
        truth[name]=dict(np.load(folder/'privileged.npz'));ctrl[name]=dict(np.load(folder/'controller.npz'))
        signed_load=o['independent_object_wrenches'][:,:,2]
        row=dict(episode=name,scenario=name[0],pitch_excursion_deg=float(np.rad2deg(np.max(np.abs(o['pitch']-o['pitch'][0])))),
                 final_pitch_change_deg=float(np.rad2deg(o['pitch'][-1]-o['pitch'][0])),
                 human_force_peak_N=float(np.max(np.linalg.norm(truth[name]['human_applied_wrench'][:,:3],axis=1))),
                 left_support_Fz_min_N=float(signed_load[:,0].min()),left_support_Fz_max_N=float(signed_load[:,0].max()),
                 right_support_Fz_min_N=float(signed_load[:,1].min()),right_support_Fz_max_N=float(signed_load[:,1].max()),
                 load_difference_Fz_peak_N=float(np.max(np.abs(signed_load[:,0]-signed_load[:,1]))),
                 drive_force_peak_N=float(np.linalg.norm(ctrl[name]['executed_virtual_drive_wrenches'][:,:,:3],axis=2).max()),
                 drive_torque_peak_Nm=float(np.linalg.norm(ctrl[name]['executed_virtual_drive_wrenches'][:,:,3:],axis=2).max()),
                 panel_z_min_m=float(o['object_pose'][:,2].min()),panel_z_max_m=float(o['object_pose'][:,2].max()))
        for side in ('left','right'):
            row[side+'_cop_shift_max_mm']=float(1000*np.linalg.norm(o['contact_center_'+side]-o['contact_center_'+side][0],axis=2).max())
            row[side+'_slip_max_mm_s']=float(o['slip_speed_'+side].max()*1000)
        rows.append(row)
    save_csv(out/'episode_summary.csv',rows)
    pairs,details=match(data);save_csv(out/'matched_pairs.csv',pairs);save_csv(out/'matching_components.csv',details)
    best={level:min([p for p in pairs if p['matching_level']==level],key=lambda p:(not p['qualified'],p['score'])) for level in ('S1','S2')}
    summary=dict(episodes=len(data),samples=sum(len(o['timestamp']) for o in data.values()),scenario_ranges={},
                 matching=dict(window_samples=WINDOW,duration_s=.15,start_range_s=[.85,2.35],stride_s=.05,candidates_per_block=len(STARTS)**2,
                               S1_tolerances=dict(zip(feature_labels('S1'),tolerance('S1').tolist())),S2_tolerances=dict(zip(feature_labels('S2'),tolerance('S2').tolist()))),
                 best_pairs=best,expert_feasibility={'status':'not_completed','valid_expert_labels':0,'conflicting_action_history_test':'unavailable without expert'},
                 S1_to_S2_scope='independent load distribution retained; not a matched proof of intent or force-target gain',
                 S2_to_S3_scope='spatial contact features present; conditional additional information not established')
    for level in ('S1','S2'):
        pr=[p for p in pairs if p['matching_level']==level]
        summary['matching'][level]=dict(qualified_blocks=sum(p['qualified'] for p in pr),blocks=len(pr),qualified_candidates=sum(p['qualified_candidate_pairs'] for p in pr),
                                       half_tolerance_blocks=sum(p['qualified_at_0.5']>0 for p in pr),quarter_tolerance_blocks=sum(p['qualified_at_0.25']>0 for p in pr))
    for s in 'ABC':
        rs=[r for r in rows if r['scenario']==s]
        summary['scenario_ranges'][s]={k:[min(r[k] for r in rs),max(r[k] for r in rs)] for k in rows[0] if k not in ('episode','scenario')}
    (out/'analysis.json').write_text(json.dumps(summary,indent=2)+'\n')
    plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.2})
    colors=dict(A='#2078b4',B='#29904a',C='#d86822')
    def finish(fig,name):
        fig.tight_layout();fig.savefig(figs/name,dpi=160);plt.close(fig)
    fig,axes=plt.subplots(6,2,figsize=(12,13),sharex=True)
    for col,side in enumerate(('left','right')):
        for k in range(6):
            ax=axes[k,col]
            for s in 'ABC':
                o=data[s+'_000'];ax.plot(o['timestamp'],o['wrist_ft_'+side][:,k],color=colors[s],label=s)
            ax.set_ylabel(W_LABELS[k]+(' [N]' if k<3 else ' [N m]'))
        axes[0,col].set_title(side.title()+' gripper');axes[0,col].legend(ncol=3);axes[-1,col].set_xlabel('Time [s]')
    fig.suptitle('Block 0: independent reconstructed wrist F/T (panel on gripper)');finish(fig,'01_independent_ft.png')
    fig,axes=plt.subplots(3,1,figsize=(11,9),sharex=True)
    for name,o in data.items():
        s=name[0];alpha=.95 if name.endswith('000') else .2
        axes[0].plot(o['timestamp'],np.rad2deg(o['pitch']),color=colors[s],alpha=alpha,label=s if name.endswith('000') else None)
        axes[1].plot(o['timestamp'],o['object_velocity'][:,1],color=colors[s],alpha=alpha)
        axes[2].plot(o['timestamp'],o['object_pose'][:,2],color=colors[s],alpha=alpha)
    for ax,label in zip(axes,['Pitch [deg]','Angular velocity y [rad/s]','Panel COM z [m]']):ax.set_ylabel(label)
    axes[0].legend();axes[-1].set_xlabel('Time [s]');fig.suptitle('All 30 episodes: compliant support allows translation as well as pitch');finish(fig,'02_object_motion.png')
    fig,axes=plt.subplots(4,3,figsize=(12,12),layout='constrained')
    vmax=max(data[s+'_000']['planar_tactile_'+side][180,:,0].max() for s in 'ABC' for side in ('left','right'))
    for row,(side,pad) in enumerate((('left',0),('left',1),('right',0),('right',1))):
        for col,s in enumerate('ABC'):
            im=axes[row,col].imshow(data[s+'_000']['planar_tactile_'+side][180,pad,0],origin='lower',interpolation='nearest',vmin=0,vmax=vmax,extent=[-45,45,-70,70],aspect='auto')
            axes[row,col].set_title(s+' / '+side+' / '+('upper' if pad==0 else 'lower'));axes[row,col].grid(False);axes[row,col].set_xlabel('pad x [mm]');axes[row,col].set_ylabel('pad y [mm]')
    fig.colorbar(im,ax=axes,label='Normal force [N/bin]');fig.suptitle('Block 0 at 1.80 s: four physical pads, sparse maps without interpolation');fig.savefig(figs/'03_tactile.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(4,1,figsize=(11,11),sharex=True)
    for s in 'ABC':
        name=s+'_000';o=data[name];t=o['timestamp'];w=o['independent_object_wrenches'];ct=ctrl[name]
        axes[0].plot(t,w[:,0,2],color=colors[s],label=s+' left');axes[0].plot(t,w[:,1,2],'--',color=colors[s],label=s+' right')
        axes[1].plot(t,ct['executed_virtual_drive_wrenches'][:,0,2],color=colors[s]);axes[1].plot(t,ct['executed_virtual_drive_wrenches'][:,1,2],'--',color=colors[s])
        axes[2].plot(t,truth[name]['human_applied_wrench'][:,2],color=colors[s],label=s+' human');axes[2].plot(t,truth[name]['external_disturbance_wrench'][:,2],'--',color=colors[s],label=s+' environment')
        axes[3].plot(t,w[:,0,2]-w[:,1,2],color=colors[s],label=s)
    labels=['Realized panel support Fz [N]','Virtual drive Fz [N]','Privileged external Fz [N]','Left minus right Fz [N]']
    for ax,label in zip(axes,labels):ax.set_ylabel(label)
    axes[0].legend(ncol=3);axes[2].legend(ncol=3);axes[-1].set_xlabel('Time [s]')
    fig.suptitle('Block 0: measured support, executed drives and privileged forces are distinct');finish(fig,'04_force_budget.png')
    fig,axes=plt.subplots(2,2,figsize=(12,7),sharex=True)
    for g,side in enumerate(('left','right')):
        for pad in (0,1):
            for s in 'ABC':
                o=data[s+'_000'];axes[g,pad].plot(o['timestamp'],1000*o['contact_center_'+side][:,pad,0],color=colors[s],label=s)
            axes[g,pad].set_title(side+' / '+('upper' if pad==0 else 'lower'));axes[g,pad].set_ylabel('Contact center x [mm]');axes[g,pad].legend()
    for ax in axes[-1]:ax.set_xlabel('Time [s]')
    fig.suptitle('Block 0: force-weighted contact centers');finish(fig,'05_contact_centers.png')
    p=best['S1'];oa,oc=data[p['A']],data[p['C']];sa=slice(round(p['A_start_s']*100),round(p['A_start_s']*100)+WINDOW);sc=slice(round(p['C_start_s']*100),round(p['C_start_s']*100)+WINDOW)
    fig,axes=plt.subplots(6,3,figsize=(15,13),sharex=True)
    for k,ax in enumerate(axes.flat):
        for o,sl,s in ((oa,sa,'A'),(oc,sc,'C')):ax.plot(np.arange(WINDOW)*.01,features(o,'S1')[sl,k],color=colors[s],label=s)
        err=np.sqrt(np.mean((features(oa,'S1')[sa,k]-features(oc,'S1')[sc,k])**2))
        ax.set_title(f'{feature_labels("S1")[k]}: RMSE {err:.3g} / tol {tolerance("S1")[k]:.3g}',fontsize=9)
    axes[0,0].legend()
    for ax in axes[-1]:ax.set_xlabel('Window time [s]')
    fig.suptitle(f'Best S1 attempt {p["A"]}@{p["A_start_s"]:.2f}s vs {p["C"]}@{p["C_start_s"]:.2f}s; qualified={p["qualified"]}\nUnits: force N, torque N m, position m, rotation rad, angular rad/s, linear m/s');finish(fig,'06_matched_S1.png')
    fig,axes=plt.subplots(1,3,figsize=(14,4.5),layout='constrained')
    for ax,level in zip(axes[:2],('S1','S2')):
        pr=[r for r in pairs if r['matching_level']==level];ax.bar(np.arange(len(pr)),[max(r['S1_max_tolerance_ratio'],r[level+'_max_tolerance_ratio']) for r in pr]);ax.axhline(1,color='r',linestyle='--',label='Every component must be <= 1')
        ax.set_xlabel('Nuisance block (attempted pair)');ax.set_ylabel('Worst RMSE / tolerance');ax.set_title(level+' best attempts');ax.legend(fontsize=8)
    pr=[r for r in pairs if r['matching_level']=='S2'];axes[2].bar(np.arange(len(pr)),[r['normal_rmse_N'] for r in pr],color='#999999');axes[2].set_title('S3 tactile residual (unqualified pairs)');axes[2].set_xlabel('Block');axes[2].set_ylabel('Normal grid RMSE [N/bin]')
    fig.suptitle('Residual tactile differences do not establish conditional information gain');fig.savefig(figs/'07_ablation_matching.png',dpi=160);plt.close(fig)
    print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
