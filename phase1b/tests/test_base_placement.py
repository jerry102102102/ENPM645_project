"""Candidate layouts cannot change joint limits, grasp geometry, or the active model."""
import copy
from pathlib import Path
import tempfile
import unittest
import mujoco
import numpy as np
from phase1b.base_placement import place, export_model
from phase1b.simulation import PandaWorld

class BasePlacementTests(unittest.TestCase):
    def test_candidate_export_preserves_hardware_and_grasps(self):
        w=PandaWorld()
        original_pos=w.m.body_pos.copy()
        placement=(.25,.45,.05,30.)
        m=copy.copy(w.m);place(m,placement)
        for sign,side in ((-1,'left'),(1,'right')):
            np.testing.assert_allclose(m.body(side+'_link0').pos,[sign*.25,.45,.05])
            np.testing.assert_allclose(m.geom(side+'_pedestal').pos,[sign*.25,.45,.025])
        np.testing.assert_array_equal(w.m.body_pos,original_pos)
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'scene.xml'
            export_model(path,w,placement,w.d.qpos)
            rebuilt=mujoco.MjModel.from_xml_path(str(path))
            for name in ('body_pos','body_quat','geom_pos','geom_size','jnt_range',
                         'actuator_ctrlrange','actuator_forcerange','site_pos','site_quat'):
                np.testing.assert_allclose(getattr(rebuilt,name),getattr(m,name),atol=1e-14,err_msg=name)
            np.testing.assert_array_equal(rebuilt.key_qpos[0],w.d.qpos)


class CandidateManualTests(unittest.TestCase):
    def test_candidate_pipeline_replay_and_scene_identity(self):
        from phase1b.human import run_human
        from phase1b.pipeline import read_episode
        from phase1b.simulation import ROOT, CONFIG
        from phase1b.hardware import sha
        config=ROOT/'phase1b/config/dual_panda_base_candidate.json'
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            first=run_human(root/'first',follow_input=True,scene_config=config,duration=6.)
            second=run_human(root/'replay',mode='replay',replay=root/'first')
            self.assertEqual(first['target_parameters']['limit_deg'],90.)
            self.assertEqual(second['scene_config'],'phase1b/config/dual_panda_base_candidate.json')
            self.assertEqual(first['clearance_checks'],600)
            self.assertTrue(first['verification']['physical_sample_checks_passed'])
            a,meta=read_episode(root/'first/episode');b,_=read_episode(root/'replay/episode')
            self.assertEqual(meta['scenario_version'],'3.1.0-preview.1')
            self.assertEqual(meta['hardware_version'],'2.1.0-preview.1')
            self.assertEqual(meta['config_path'],second['scene_config'])
            self.assertFalse(Path(meta['hardware_path']).is_absolute())
            self.assertNotEqual(meta['config_hash'],sha(CONFIG))
            for section in a:
                for key in a[section]:
                    np.testing.assert_array_equal(a[section][key],b[section][key])
            with self.assertRaisesRegex(ValueError,'recorded scene'):
                run_human(root/'override',mode='replay',replay=root/'first',scene_config=CONFIG)
            with self.assertRaisesRegex(ValueError,'requires --follow-input'):
                run_human(root/'no_follow',scene_config=config)

    def test_candidate_custom_controller_entry_and_replay(self):
        from phase1b.human import run_human
        from phase1b.pipeline import read_episode
        from phase1b.simulation import ROOT
        config=ROOT/'phase1b/config/dual_panda_base_candidate.json'
        obj='phase1b.interfaces:ReferenceOnlyObjectAdapter'
        arm='phase1b.interfaces:PositionHoldDualArmAdapter'
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            first=run_human(root/'custom',scene_config=config,duration=5.,
                            object_controller=obj,dual_arm_controller=arm)
            repeated=run_human(root/'replay',mode='replay',replay=root/'custom')
            self.assertEqual(repeated['object_controller'],obj)
            self.assertEqual(repeated['dual_arm_controller'],arm)
            self.assertFalse(first['follow_input'])
            self.assertEqual(first['target_parameters']['limit_deg'],10.)
            self.assertEqual(first['clearance_checks'],500)
            a,_=read_episode(root/'custom/episode');b,_=read_episode(root/'replay/episode')
            for section in a:
                for key in a[section]: np.testing.assert_array_equal(a[section][key],b[section][key])
            np.testing.assert_array_equal(a['decisions']['mode'],'UNCERTAIN')
            self.assertFalse(a['supervision']['expert_label_valid'].any())
            self.assertNotIn('human_goal_angle',a['observations'])
            with self.assertRaisesRegex(ValueError,'recorded controller'):
                run_human(root/'wrong',mode='replay',replay=root/'custom',object_controller='wrong:Class')
            with self.assertRaisesRegex(ValueError,'requires --follow-input'):
                run_human(root/'wrong_delay',scene_config=config,object_controller=obj,
                          dual_arm_controller=arm,follow_delay_ms=200.)

    def test_candidate_benchmark_uses_same_scene_and_guards(self):
        from phase1b.benchmark import run_benchmark, BenchmarkWorld, physical_guard
        from phase1b.simulation import ROOT
        from phase1b.pipeline import read_episode
        config=ROOT/'phase1b/config/dual_panda_base_candidate.json'
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)/'benchmark'
            report=run_benchmark(root,cases=('T0',),scene_config=config)
            _,meta=read_episode(root/'T0')
            self.assertEqual(meta['scenario_version'],'3.1.0-preview.1')
            self.assertEqual(report['runs']['T0']['clearance_checks'],400)
            self.assertFalse(report['runs']['T0']['verification']['assistance_tracking_validated'])
        world=BenchmarkWorld(config_path=config)
        world.restore(world.prepare())
        physical_guard(world)
        world.d.qpos[world.qa[0,0]]=world.m.jnt_range[world.arm_joints[0,0],0]+np.deg2rad(4)
        with self.assertRaisesRegex(RuntimeError,'5-degree'):
            physical_guard(world)

    def test_clearance_bound_detects_overlapping_arms_and_reserve_is_planning_only(self):
        from phase1b.simulation import ROOT
        from phase1b.human_follow import InterArmClearance,PositionFollowArms
        w=PandaWorld(config_path=ROOT/'phase1b/config/dual_panda_base_candidate.json')
        limits=w.m.jnt_range.copy()
        o,_=w.snapshot()
        planner=PositionFollowArms(w.m,w.qtarget,np.r_[o['object_position'],o['object_orientation']],joint_reserve_deg=5.)
        np.testing.assert_array_equal(w.m.jnt_range,limits)
        np.testing.assert_allclose(planner.m.jnt_range[w.arm_joints,0],limits[w.arm_joints,0]+np.deg2rad(5))
        bound=InterArmClearance(w.m)
        self.assertGreaterEqual(bound.lower_bound(w.d),.1)
        w.m.body('right_link0').pos[:]=w.m.body('left_link0').pos
        w.m.body('right_link0').quat[:]=w.m.body('left_link0').quat
        w.d.qpos[w.qa[1]]=w.d.qpos[w.qa[0]]
        mujoco.mj_forward(w.m,w.d)
        self.assertLess(bound.lower_bound(w.d),.1)

if __name__=='__main__':
    unittest.main()
