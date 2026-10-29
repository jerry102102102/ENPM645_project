"""Descriptive, episode-blocked analysis. No labels enter sensor generation."""
from __future__ import annotations
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
LABELS = ['Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz', 'pitch', 'omega_y']
# Fixed engineering tolerances, not fitted sensor noise estimates.
TOL = np.array([.5, .5, .5, .05, .05, .05, np.deg2rad(1), .03])
WIN = 15  # 150 ms at 100 Hz, compare every sample rather than window means.


def write_csv(path, rows):
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader(); writer.writerows(rows)


def features(o):
    return np.column_stack([o['wrist_ft'], o['angle'], o['object_velocity'][:, 1]])


def compare(a, c, ai, ci):
    sa, sc = slice(ai, ai+WIN), slice(ci, ci+WIN)
    rmse = np.sqrt(np.mean((features(a)[sa]-features(c)[sc])**2, axis=0))
    ma, mc = a['planar_contact_map'][sa, :, 0], c['planar_contact_map'][sc, :, 0]
    na, nc = ma/ma.sum(axis=(2, 3), keepdims=True), mc/mc.sum(axis=(2, 3), keepdims=True)
    return dict(score=float(np.sqrt(np.mean((rmse/TOL)**2))),
                strict_match=bool(np.all(rmse <= TOL)),
                **{k+'_rmse': float(v) for k, v in zip(LABELS, rmse)},
                planar_normal_rmse_N=float(np.sqrt(np.mean((ma-mc)**2))),
                normalized_normal_L1=float(np.mean(np.sum(np.abs(na-nc), axis=(2, 3)))),
                planar_shear_rmse_N=float(np.sqrt(np.mean((a['planar_contact_map'][sa, :, 1:]-c['planar_contact_map'][sc, :, 1:])**2))),
                plugin_rmse_N=float(np.sqrt(np.mean((a['tactile_map'][sa]-c['tactile_map'][sc])**2))),
                cop_rmse_mm=float(1000*np.sqrt(np.mean((a['contact_center'][sa]-c['contact_center'][sc])**2))),
                normal_load_rmse_N=float(np.sqrt(np.mean((a['normal_load'][sa]-c['normal_load'][sc])**2))),
                max_contact_count_difference=int(np.max(np.abs(a['contact_count'][sa]-c['contact_count'][sc]))),
                position_rmse_mm=float(1000*np.sqrt(np.mean((a['object_pose'][sa, :3]-c['object_pose'][sc, :3])**2))),
                roll_yaw_angular_velocity_rmse=float(np.sqrt(np.mean((a['object_velocity'][sa][:, [0, 2]]-c['object_velocity'][sc][:, [0, 2]])**2))))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=ROOT/'results/pilot')
    args = parser.parse_args(); out = args.output; figs = out/'figures'; figs.mkdir(exist_ok=True)
    manifest = json.loads((out/'manifest.json').read_text())
    data, metadata, rows = {}, {}, []
    for item in manifest:
        name = item['episode']; folder = out/'data'/name
        o = dict(np.load(folder/'observations.npz')); m = json.loads((folder/'metadata.json').read_text())
        assert len(o['time']) == 400 and np.all(np.isfinite(o['wrist_ft']))
        data[name], metadata[name] = o, m
        rows.append(dict(episode=name, scenario=m['scenario'], seed=m['seed'],
                        pitch_excursion_deg=float(np.rad2deg(np.max(np.abs(o['angle']-o['angle'][0])))),
                        final_pitch_change_deg=float(np.rad2deg(o['angle'][-1]-o['angle'][0])),
                        force_delta_max_N=float(np.max(np.linalg.norm(o['wrist_ft'][:, :3]-o['wrist_ft'][0, :3], axis=1))),
                        torque_delta_max_Nm=float(np.max(np.linalg.norm(o['wrist_ft'][:, 3:]-o['wrist_ft'][0, 3:], axis=1))),
                        cop_shift_max_mm=float(1000*np.max(np.linalg.norm(o['contact_center']-o['contact_center'][0], axis=2))),
                        slip_max_mm_s=float(1000*o['slip_speed'].max()),
                        min_load_N=float(o['normal_load'].min()), max_load_N=float(o['normal_load'].max()),
                        min_contacts=int(o['contact_count'].min()),max_contacts=int(o['contact_count'].max())))
    write_csv(out/'episode_summary.csv', rows)
    pairs = []; best_ids = {}
    for name, a in data.items():
        if not name.startswith('A_'): continue
        cname = name.replace('A_', 'C_'); c = data[cname]
        # No baseline/after-disturbance windows: compare active response only.
        # Same seed/block fixes stiffness, damping, friction, preload and initial state.
        starts = np.arange(85, 236, 5)
        aw = np.array([features(a)[s:s+WIN]/TOL for s in starts])
        cw = np.array([features(c)[s:s+WIN]/TOL for s in starts])
        per_feature = np.mean((aw[:, None]-cw[None, :])**2, axis=2)
        scores = np.sqrt(per_feature.mean(axis=2))
        valid = np.all(per_feature <= 1., axis=2)
        # Prefer a qualifying pair if any exists, then the lowest RMS score.
        chosen = np.where(valid, scores, np.inf) if valid.any() else scores
        ix, jx = np.unravel_index(np.argmin(chosen), scores.shape)
        ai, ci = int(starts[ix]), int(starts[jx])
        pair = dict(A=name, C=cname, A_start_s=ai/100, C_start_s=ci/100,
                    qualifying_candidate_pairs=int(valid.sum()), **compare(a, c, ai, ci))
        for factor in (.5, .25):
            pair[f'candidate_pairs_at_{factor}_tolerance'] = int(np.all(per_feature <= factor**2, axis=2).sum())
        pairs.append(pair); best_ids[name] = ai, ci
    write_csv(out/'matched_pairs.csv', pairs)
    summary = dict(episodes=len(rows), samples=sum(len(o['time']) for o in data.values()),
                   scenarios={}, matching=dict(window_samples=WIN, window_duration_s=.15, candidate_start_s=[.85,2.35],
                   stride_s=.05, tolerances=dict(zip(LABELS,TOL.tolist())),
                   blocks=len(pairs), strict_matched_blocks=sum(p['strict_match'] for p in pairs),
                   blocks_at_half_tolerance=sum(p['candidate_pairs_at_0.5_tolerance']>0 for p in pairs),
                   blocks_at_quarter_tolerance=sum(p['candidate_pairs_at_0.25_tolerance']>0 for p in pairs)),
                   validation=dict(max_acceleration_m_s2=max(m['validation']['max_linear_acceleration_m_s2'] for m in metadata.values()),
                   max_penetration_mm=1000*max(m['validation']['max_penetration_m'] for m in metadata.values()),
                   contact_loss_steps=sum(m['validation']['contact_loss_steps'] for m in metadata.values()),
                   floor_contact_steps=sum(m['validation']['floor_contact_steps'] for m in metadata.values()),
                   warnings=sum(sum(m['validation']['warning_counts']) for m in metadata.values()),
                   max_balance_residual_N=max(m['validation']['max_balance_residual_N'] for m in metadata.values())))
    for scenario in 'ABC':
        rs = [r for r in rows if r['scenario']==scenario]
        summary['scenarios'][scenario] = {k: [min(r[k] for r in rs), max(r[k] for r in rs)] for k in rows[0] if k not in ('episode','scenario','seed')}
    best = min(pairs, key=lambda p: (not p['strict_match'], p['score']))
    summary['best_pair'] = best
    (out/'analysis.json').write_text(json.dumps(summary, indent=2)+'\n')
    plt.rcParams.update({'font.size': 10, 'axes.grid': True, 'grid.alpha': .2, 'figure.dpi': 120})
    colors = dict(A='#2078b4', B='#29904a', C='#d86822')
    def finish(fig, name):
        fig.tight_layout(); fig.savefig(figs/name, dpi=170); plt.close(fig)
    fig, axes = plt.subplots(3,2, figsize=(11,8), sharex=True)
    for k, ax in enumerate(axes.T.flat):
        for s in 'ABC':
            o=data[s+'_000']; ax.plot(o['time'], o['wrist_ft'][:,k], label=s, color=colors[s])
        ax.set_ylabel(LABELS[k]+(' [N]' if k<3 else ' [N m]'));ax.legend(ncol=3)
    axes[-1,0].set_xlabel('Time [s]'); axes[-1,1].set_xlabel('Time [s]')
    fig.suptitle('Block 0: reconstructed panel-on-gripper wrench (wrist frame)')
    finish(fig,'01_wrench.png')
    fig, axes=plt.subplots(2,1,figsize=(10,7),sharex=True)
    for s in 'ABC':
        for name,o in data.items():
            if name[0]!=s:continue
            axes[0].plot(o['time'],np.rad2deg(o['angle']),color=colors[s],alpha=.25)
            axes[1].plot(o['time'],o['object_velocity'][:,1],color=colors[s],alpha=.25)
        o=data[s+'_000'];axes[0].plot(o['time'],np.rad2deg(o['angle']),color=colors[s],label=s)
        axes[1].plot(o['time'],o['object_velocity'][:,1],color=colors[s],label=s)
    axes[0].set_ylabel('Pitch [deg]');axes[1].set_ylabel('Angular velocity y [rad/s]');axes[1].set_xlabel('Time [s]')
    axes[0].legend();fig.suptitle('All 30 episodes (block 0 emphasized)')
    finish(fig,'02_motion.png')
    # Grid cells contain summed force, never interpolated pressure.
    fig,axes=plt.subplots(2,3,figsize=(11,6),layout='constrained')
    vmax=max(data[s+'_000']['planar_contact_map'][180,:,0].max() for s in 'ABC')
    for j,s in enumerate('ABC'):
        for pad in range(2):
            ax=axes[pad,j];im=ax.imshow(data[s+'_000']['planar_contact_map'][180,pad,0],origin='lower',interpolation='nearest',vmin=0,vmax=vmax,cmap='viridis',extent=[-45,45,-70,70],aspect='auto')
            ax.set_title(f'{s} / {"upper" if pad==0 else "lower"}');ax.set_xlabel('Pad x [mm]');ax.set_ylabel('Pad y [mm]');ax.grid(False)
    fig.colorbar(im,ax=axes,label='Normal contact force per planar bin [N]');fig.suptitle('Block 0, t = 1.80 s: sparse physical contacts, no interpolation')
    fig.savefig(figs/'03_tactile.png',dpi=170);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(11,7),sharex=True)
    for pad in range(2):
        for s in 'ABC':
            o=data[s+'_000'];axes[0,pad].plot(o['time'],1000*o['contact_center'][:,pad,0],color=colors[s],label=s)
            axes[1,pad].plot(o['time'],o['normal_load'][:,pad],color=colors[s],label=s)
        axes[0,pad].set_title('Upper pad' if pad==0 else 'Lower pad');axes[0,pad].set_ylabel('Force-weighted contact x [mm]')
        axes[1,pad].set_ylabel('Total normal force [N]');axes[1,pad].set_xlabel('Time [s]');axes[0,pad].legend()
    fig.suptitle('Block 0: contact center and load (4 contacts per pad throughout)')
    finish(fig,'04_contact_center.png')
    a,c=data[best['A']],data[best['C']];ai,ci=best_ids[best['A']];ss=[slice(ai,ai+WIN),slice(ci,ci+WIN)];tt=np.arange(WIN)*.01
    fig,axes=plt.subplots(4,2,figsize=(11,10),sharex=True)
    for k,ax in enumerate(axes.flat):
        for o,sl,s in zip((a,c),ss,'AC'):ax.plot(tt,features(o)[sl,k],color=colors[s],label=s)
        ax.set_ylabel(LABELS[k]+(' [N]' if k<3 else ' [N m]' if k<6 else ' [rad]' if k==6 else ' [rad/s]'))
        ax.set_title(f'RMSE {best[LABELS[k]+"_rmse"]:.4g}; tolerance {TOL[k]:.4g}',fontsize=9)
    axes[0,0].legend();axes[-1,0].set_xlabel('Window time [s]');axes[-1,1].set_xlabel('Window time [s]')
    fig.suptitle(f'Best A/C pair: {best["A"]} @ {ai/100:.2f}s, {best["C"]} @ {ci/100:.2f}s\nStrict match: {best["strict_match"]}; normalized RMS = {best["score"]:.3f}')
    finish(fig,'05_matched_wrench.png')
    fig,axes=plt.subplots(2,3,figsize=(11,6),layout='constrained')
    am=a['planar_contact_map'][ss[0],:,0].mean(axis=0);cm=c['planar_contact_map'][ss[1],:,0].mean(axis=0);dm=am-cm
    for pad in range(2):
        for j,x in enumerate((am,cm,dm)):
            lim=np.max(np.abs(dm)) if j==2 else max(am.max(),cm.max())
            im=axes[pad,j].imshow(x[pad],origin='lower',interpolation='nearest',cmap='coolwarm' if j==2 else 'viridis',vmin=-lim if j==2 else 0,vmax=lim)
            axes[pad,j].grid(False);axes[pad,j].set_title(('Upper ' if pad==0 else 'Lower ')+('A','C','A minus C')[j]);fig.colorbar(im,ax=axes[pad,j],label='N/bin')
    fig.suptitle('Best pair: window-mean sparse normal maps; residual is not proof of intent')
    fig.savefig(figs/'06_matched_tactile.png',dpi=170);plt.close(fig)
    print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
