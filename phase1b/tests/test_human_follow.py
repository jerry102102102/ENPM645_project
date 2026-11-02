"""Known-input motion baseline: contact, stop/reverse, causal routing and replay."""

import tempfile
from pathlib import Path
import unittest

import numpy as np

from phase1b.benchmark import physical_guard
from phase1b.human import HandTargets, HumanWorld, run_human
from phase1b.human_follow import KnownInputPolicy, KnownInputObjectReference, PositionFollowArms, DelayedHandReference, matrix, scan_pitch
from phase1b.interfaces import ObjectCommand
from phase1b.pipeline import controller_input, read_episode, run_episode


def pitch(quaternions):
    return np.array([np.arctan2(-matrix(q)[2, 0], matrix(q)[0, 0]) for q in quaternions])


class FollowTests(unittest.TestCase):
    def test_reference_delay_startup_causality_zero_delay_and_tick_validation(self):
        hand=HandTargets()
        hand.add(.1,'ROTATE_POS',10.)
        hand.add(.2,'HOLD')
        hand.add(.3,'ROTATE_NEG',-10.)
        delayed=DelayedHandReference(hand,200.)
        immediate=DelayedHandReference(hand,0.)
        for tick in range(70):
            t=tick*.01
            self.assertEqual(immediate.sample(t),hand.sample(t))
            actual=delayed.sample(t)
            self.assertEqual(delayed.sample(t),actual)  # policy/object share one frozen sample
            if tick<20:
                self.assertEqual(actual[:2],(0.,0.))
                self.assertFalse(delayed.records[-1]['reference_available'])
                self.assertTrue(np.isnan(delayed.records[-1]['source_sample_timestamp']))
            else:
                expected=hand.sample((tick-20)*.01)
                np.testing.assert_allclose(actual[:2],expected[:2],atol=1e-10)
                self.assertEqual(actual[2],expected[2])
                self.assertAlmostEqual(delayed.records[-1]['reference_age_s'],.2)
        self.assertEqual(len(delayed.records),70)
        for delay in (-1,5,np.nan,np.inf,True):
            with self.assertRaises(ValueError): DelayedHandReference(hand,delay)
        with self.assertRaises(ValueError): delayed.sample(.685)
        with self.assertRaises(ValueError): delayed.sample(.1)
        with self.assertRaises(ValueError): delayed.sample(.71)
        between_ticks=HandTargets()
        between_ticks.add(.102,'ROTATE_POS',10.)
        sampled=DelayedHandReference(between_ticks,200.)
        for tick in range(32): sampled.sample(tick*.01)
        self.assertEqual(sampled.records[30]['follower_mode'],'HOLD')
        self.assertEqual(sampled.records[31]['follower_mode'],'ROTATE_POS')
        self.assertAlmostEqual(sampled.records[31]['source_sample_timestamp'],.11)
        self.assertAlmostEqual(sampled.records[31]['source_event_timestamp'],.102)

    def test_delay200_physical_pipeline_timing_and_exact_replay(self):
        from phase1b.simulation import ROOT
        config=ROOT/'phase1b/config/dual_panda_base_candidate.json'
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            report=run_human(root/'delayed',follow_input=True,scene_config=config,follow_delay_ms=200.,duration=6.)
            repeated=run_human(root/'replay',mode='replay',replay=root/'delayed')
            self.assertTrue(report['verification']['physical_sample_checks_passed'])
            self.assertEqual(repeated['follow_delay_ms'],200.)
            a,_=read_episode(root/'delayed/episode');b,_=read_episode(root/'replay/episode')
            for section in a:
                for key in a[section]: np.testing.assert_array_equal(a[section][key],b[section][key])
            with np.load(root/'delayed/follower_timing.npz') as timing, np.load(root/'replay/follower_timing.npz') as replayed:
                for key in timing.files: np.testing.assert_array_equal(timing[key],replayed[key])
                np.testing.assert_array_equal(timing['reference_available'][:20],False)
                np.testing.assert_array_equal(timing['reference_available'][20:],True)
                np.testing.assert_allclose(timing['reference_age_s'][20:],.2,atol=1e-12)
                np.testing.assert_allclose(timing['arm_reference_age_s'][20:],np.tile(.2+np.arange(5)*.002,(580,1)),atol=1e-12)
                np.testing.assert_allclose(timing['follower_target_angle_deg'][20:],timing['human_target_angle_deg'][:-20],atol=1e-12)
                np.testing.assert_array_equal(timing['follower_mode'][20:],timing['human_mode'][:-20])
                np.testing.assert_array_equal(timing['follower_mode'],a['decisions']['mode'])
                np.testing.assert_array_equal(timing['arm_command_timestamps'],a['controller']['command_timestamp'])
            self.assertFalse(a['supervision']['expert_label_valid'].any())
            with self.assertRaisesRegex(ValueError,'recorded follow delay'):
                run_human(root/'override',mode='replay',replay=root/'delayed',follow_delay_ms=100.)

    def test_demo_follows_stops_reverses_replays_and_preserves_label_boundary(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            report = run_human(root / 'follow', duration=6., follow_input=True)
            repeated = run_human(root / 'replay', mode='replay', replay=root / 'follow')
            self.assertTrue(repeated['follow_input'])
            self.assertTrue(report['verification']['physical_sample_checks_passed'])
            a, meta = read_episode(root / 'follow/episode')
            b, _ = read_episode(root / 'replay/episode')
            for section in a:
                for key in a[section]:
                    np.testing.assert_array_equal(a[section][key], b[section][key])
            self.assertTrue(meta['experiment']['known_human_input_baseline'])
            p = pitch(a['observations']['object_orientation'])
            p = np.rad2deg(p - p[0])
            self.assertAlmostEqual(p[280], 7.5, delta=.1)
            self.assertAlmostEqual(p[-1], -7.5, delta=.1)
            self.assertLess(np.ptp(p[220:300]), .1)
            self.assertLess(np.ptp(p[480:]), .1)
            # The physical panel settles after the hand target stops; it is not teleported.
            self.assertLess(p[150], p[220])
            self.assertGreater(np.max(np.abs(a['observations']['arm_joint_position']
                                           - a['observations']['arm_joint_position'][0])), .05)
            commands = a['controller']['arm_position_target'].reshape(-1, 2, 7)
            self.assertLessEqual(np.max(np.abs(np.diff(commands, axis=0))), .001 + 1e-12)
            self.assertFalse(a['controller']['assist_reference_executed'].any())
            self.assertFalse(a['supervision']['expert_label_valid'].any())
            self.assertTrue(np.isnan(a['supervision']['expert_assist_torque_ref']).all())
            np.testing.assert_array_equal(a['controller']['controller_mode'], 'known_input_position_follow')
            np.testing.assert_array_equal(a['decisions']['assist_torque_y_nm'], 0.)
            self.assertNotIn('ground_truth_intent', a['observations'])

    def test_full_input_range_and_isolated_ik(self):
        world = HumanWorld()
        saved = world.prepare()
        world.restore(saved)
        hand = HandTargets(limit_deg=60.)
        for event in ((1., 'ROTATE_POS', 60.), (6., 'HOLD', None),
                      (7., 'ROTATE_NEG', -60.), (16., 'HOLD', None),
                      (17., 'ROTATE_POS', 60.), (19., 'HOLD', None),
                      (20., 'ROTATE_NEG', -60.), (20.5, 'ROTATE_POS', 0.)):
            hand.add(*event)
        world.attach(hand, 'scripted_input_test')
        initial, _ = world.snapshot()
        pose = np.r_[initial['object_position'], initial['object_orientation']]
        execution_qpos = world.d.qpos.copy()
        execution_limits = world.m.jnt_range.copy()
        arm = PositionFollowArms(world.m, world.qtarget, pose)
        np.testing.assert_array_equal(world.d.qpos, execution_qpos)
        with self.assertRaisesRegex(ValueError, 'zero assistance wrench'):
            arm.step(ObjectCommand(0., 'HOLD', pose, np.zeros(6), np.array([0.,0.,0.,0.,.01,0.])),
                     controller_input(world, initial))
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'envelope'
            result = run_episode(path, 'B', 25., world=world, step_guard=physical_guard, history_length=1,
                policy=KnownInputPolicy(hand), object_controller=KnownInputObjectReference(hand, pose),
                dual_arm_controller=arm, episode_context={'known_human_input_baseline': True})
            self.assertTrue(result['physical_sample_checks_passed'])
            data, _ = read_episode(path)
            p = pitch(data['observations']['object_orientation'])
            p = np.rad2deg(p - p[0])
            self.assertAlmostEqual(p[690], 60., delta=.2)
            self.assertAlmostEqual(p[1690], -60., delta=.2)
            self.assertAlmostEqual(p[1990], -30., delta=.2)
            self.assertAlmostEqual(p[-1], 0., delta=.2)
            # Existing physical contract: no hard joint-limit violation. The separate
            # 5-degree comfort-margin probe failed and is documented, not promoted to a pass.
            self.assertGreater(data['diagnostics']['arm_joint_limit_margin_rad'].min(), 0.)
            self.assertFalse(data['diagnostics']['arm_external_generalized_force'].any())
            self.assertFalse(data['diagnostics']['xfrc_applied_norm'].any())
            np.testing.assert_array_equal(world.m.jnt_range, execution_limits)
            np.testing.assert_array_equal(world.m.jnt_range, arm.m.jnt_range)

    def test_pitch_scan_records_sampled_branch_not_physical_certification(self):
        with tempfile.TemporaryDirectory() as temp:
            report = scan_pitch(Path(temp)/'scan', step_deg=.5, extent_deg=1.)
            self.assertEqual(report['branches']['positive']['last_passed_offset_deg'], 1.)
            self.assertEqual(report['branches']['negative']['last_passed_offset_deg'], -1.)
            self.assertIn('not a global', report['scope'])
            self.assertEqual(len(report['branches']['positive']['samples']), 3)

    def test_explicit_baseline_selection_cannot_mix_with_other_controllers(self):
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaisesRegex(ValueError, 'cannot be combined'):
                run_human(Path(temp) / 'bad', follow_input=True, dual_arm_controller='anything:Else')

    def test_operating_bound_cannot_exceed_controller_envelope(self):
        with tempfile.TemporaryDirectory() as temp:
            for follow, limit in ((True, 61.), (False, 11.), (True, float('nan')), (True, 0.)):
                with self.subTest(follow=follow, limit=limit), self.assertRaisesRegex(ValueError, 'angle limit'):
                    run_human(Path(temp)/'bad', follow_input=follow, limit_deg=limit)


if __name__ == '__main__':
    unittest.main()
