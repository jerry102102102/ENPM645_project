"""Two free-root, physically contacting grippers; external drives are fully logged."""
from pathlib import Path
import mujoco
import numpy as np
from phase0_tactile.scripts.run_experiment import rotate_y, clip_norm, smooth, parameters as v1_parameters
from phase0_v2.control.baseline import support_commands

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT/'models/dual_gripper_pilot.xml'
CENTER = np.array([0.,0.,.8])
HAND = np.array([.27,-.08,0.])
SIDES = ('left','right')


def parameters(i, seed=64600):
    p=v1_parameters(i,seed)
    p.update(settle_seconds=2., human_offset=[0.,0.,0.], gripper_separation_m=.4)
    p['ramp_duration']=max(.8,1.5*.27*p['target_angle']/p['human_target_speed_limit'])
    return p


def point_wrench(force, point, origin):
    return np.r_[force,np.cross(point-origin,force)]


class DualPilot:
    def __init__(self,p):
        self.p=p;self.m=mujoco.MjModel.from_xml_path(str(MODEL));self.d=mujoco.MjData(self.m)
        m,d=self.m,self.d
        m.opt.timestep=p['timestep'];m.geom_friction[:,0]=p['friction']
        self.panel=m.body('panel').id;self.pg=m.geom('panel_geom').id
        self.roots=[m.body(s).id for s in SIDES]
        self.pad_names=[[s+'_'+pad for pad in ('upper','lower')] for s in SIDES]
        self.pad_g=[[m.geom(n+'_pad').id for n in ns] for ns in self.pad_names]
        self.pad_b=[[m.body(n).id for n in ns] for ns in self.pad_names]
        self.gripper_masses=[float(m.body_mass[b]+sum(m.body_mass[k] for k in pads)) for b,pads in zip(self.roots,self.pad_b)]
        self.reference_rot=rotate_y(p['initial_angle'])
        self.anchors=[CENTER+self.reference_rot@np.array([x,0,0]) for x in (-.2,.2)]
        for name,pos in [('left',self.anchors[0]),('right',self.anchors[1]),('panel',CENTER)]:
            q=m.jnt_qposadr[m.joint(name+'_free').id];a=p['initial_angle']
            d.qpos[q:q+3]=pos;d.qpos[q+3:q+7]=[np.cos(a/2),0,np.sin(a/2),0]
        self.vadr=m.jnt_dofadr[m.joint('panel_free').id]
        self.hsite=m.site('human_grasp').id
        d.ctrl[:]=[-p['preload'],p['preload']]*2
        mujoco.mj_forward(m,d)

    def forces(self,t,scenario,*,disable_human=False,disable_robot=False,wrench_override=None):
        m,d,p=self.m,self.d,self.p
        mujoco.mj_fwdPosition(m,d);mujoco.mj_fwdVelocity(m,d)
        u,du=smooth(t,.7,p['ramp_duration'])
        da=p['direction']*p['target_angle'] if scenario=='A' else 0.
        goal=p['initial_angle']+da*u;r=rotate_y(goal)@HAND
        target=CENTER+r;target_v=clip_norm(np.cross([0,da*du,0],r),p['human_target_speed_limit'])
        hp=d.site_xpos[self.hsite].copy();jp=np.zeros((3,m.nv))
        mujoco.mj_jacSite(m,d,jp,None,self.hsite)
        fh=clip_norm(p['kh']*(target-hp)+p['dh']*(target_v-jp@d.qvel),p['human_force_limit'])
        if disable_human:fh[:]=0
        ep=d.xpos[self.panel]+d.xmat[self.panel].reshape(3,3)@np.array(p['disturbance_point'])
        fe=np.zeros(3)
        if scenario=='C':
            on,_=smooth(t,p['disturbance_start'],.15);off,_=smooth(t,p['disturbance_end'],.15)
            fe[2]=-p['direction']*p['disturbance_magnitude']*(on-off)
        cmd,ff,fb=support_commands(self,disabled=disable_robot)
        d.qfrc_applied[:]=0
        for b,w in zip(self.roots,cmd):mujoco.mj_applyFT(m,d,w[:3],w[3:],d.xpos[b],b,d.qfrc_applied)
        com=d.xipos[self.panel].copy()
        if wrench_override is None:
            mujoco.mj_applyFT(m,d,fh,np.zeros(3),hp,self.panel,d.qfrc_applied)
            mujoco.mj_applyFT(m,d,fe,np.zeros(3),ep,self.panel,d.qfrc_applied)
        else:
            mujoco.mj_applyFT(m,d,wrench_override[:3],wrench_override[3:],com,self.panel,d.qfrc_applied)
        mujoco.mj_forward(m,d)
        truth=dict(human_goal=goal,human_target_trajectory=target,human_target_velocity=target_v,
                   human_applied_wrench=point_wrench(fh,hp,com),external_disturbance_wrench=point_wrench(fe,ep,com),
                   human_application_point=hp,external_application_point=ep)
        controller=dict(executed_virtual_drive_wrenches=cmd,nominal_support_wrenches=ff,
                        feedback_wrenches_before_clipping=fb,executed_actuator_commands=d.ctrl.copy())
        return truth,controller

    def settle(self):
        for _ in range(round(self.p['settle_seconds']/self.m.opt.timestep)):
            self.forces(-1.,'B');mujoco.mj_step(self.m,self.d)
