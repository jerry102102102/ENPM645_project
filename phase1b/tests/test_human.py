"""Human input causality, force coupling, stop semantics and reproducible logging."""

from contextlib import nullcontext
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from phase1b.human import HandTargets, HumanWorld, run_human
from phase1b.interfaces import policy_observation
from phase1b.pipeline import read_episode


class HumanTests(unittest.TestCase):
    def test_goal_is_not_target_hold_cancels_goal_and_future_events_are_hidden(self):
        hand = HandTargets()
        hand.add(1., "ROTATE_POS", 5.)
        hand.add(1.6, "HOLD")
        hand.add(3., "ROTATE_NEG", -3.)
        self.assertEqual(hand.sample(.5), (0., 0., hand.events[0]))
        angle, speed, event = hand.sample(1.2)
        self.assertAlmostEqual(angle, 1.)
        self.assertEqual(speed, 5.)
        self.assertEqual(event["goal_offset_deg"], 5.)
        self.assertAlmostEqual(hand.sample(1.6)[0], 3.)
        self.assertEqual(hand.sample(1.6)[1], 0.)
        self.assertEqual(hand.sample(2.9)[2]["mode"], "HOLD")
        self.assertAlmostEqual(hand.sample(3.)[0], 3.)
        self.assertEqual(hand.sample(3.)[1], -5.)
        self.assertAlmostEqual(hand.sample(5.)[0], -3.)
        self.assertEqual(hand.sample(5.)[2]["mode"], "ROTATE_NEG")
        for t in (1., 1.6, 3.):
            self.assertLess(abs(hand.sample(t)[0] - hand.sample(t - 1e-7)[0]), 1e-6)

    def test_bad_event_or_limit_is_rejected(self):
        for args in ((-.002, "HOLD"), (.001, "HOLD"), (np.nan, "HOLD"),
                     (.1, "C"), (.1, "ROTATE_POS", -1), (.1, "ROTATE_NEG", 1),
                     (.1, "ROTATE_POS", 11), (.1, "HOLD", 2)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                HandTargets().add(*args)
        for speed in (0, np.nan, True):
            with self.assertRaises(ValueError):
                HandTargets(speed_deg_s=speed)
        hand = HandTargets()
        hand.add(1., "HOLD")
        with self.assertRaises(ValueError):
            hand.add(.5, "HOLD")

    def test_force_responds_to_panel_not_teleport_and_same_tick_is_not_resampled(self):
        world = HumanWorld()
        world.prepare()
        world.attach(HandTargets(), "scripted_input_test")
        poll_times = []
        world.poll = poll_times.append
        q = world.d.qpos.copy()
        world.refresh(0.)
        world.refresh(0.)
        np.testing.assert_array_equal(world.d.qpos, q)
        self.assertEqual(poll_times, [])
        world.step()
        world.refresh(.002)
        self.assertEqual(poll_times, [.002])
        force = world.truth["human_applied_force"].copy()
        target = world.truth["human_target_trajectory"].copy()
        world.d.qpos[world.m.jnt_qposadr[world.m.joint("panel_free").id]] += .001
        world.refresh(.002)
        np.testing.assert_array_equal(world.truth["human_target_trajectory"], target)
        self.assertGreater(np.linalg.norm(world.truth["human_applied_force"] - force), .01)
        np.testing.assert_array_equal(world.d.qfrc_applied[world.va], 0.)
        world.hand = HandTargets(speed_deg_s=1000)
        world.hand.add(.004, "ROTATE_POS", 5.)
        with self.assertRaisesRegex(ValueError, "speed limit"):
            world.refresh(.004)

    def test_demo_replay_all_transitions_and_private_teacher_bridge(self):
        with tempfile.TemporaryDirectory() as temp:
            first, second = Path(temp) / "first", Path(temp) / "replay"
            report = run_human(first, duration=5.)
            replay = run_human(second, mode="replay", replay=first)
            self.assertTrue(report["verification"]["physical_sample_checks_passed"])
            self.assertEqual(replay["duration_seconds"], 5.)
            a, _ = read_episode(first / "episode")
            b, _ = read_episode(second / "episode")
            for section in a:
                for key in a[section]:
                    np.testing.assert_array_equal(a[section][key], b[section][key])
            p = a["privileged"]
            self.assertEqual(set(p["ground_truth_intent"]), {"HOLD", "ROTATE_POS", "ROTATE_NEG"})
            index = 160
            np.testing.assert_allclose(p["human_target_trajectory"][index:300],
                                       np.tile(p["human_target_trajectory"][index], (140, 1)))
            self.assertLessEqual(np.linalg.norm(p["human_applied_force"], axis=1).max(), report["human_force_limit_n"])
            self.assertGreater(np.linalg.norm(p["human_applied_force"], axis=1).max(), .1)
            self.assertFalse(a["supervision"]["expert_label_valid"].any())
            np.testing.assert_array_equal(a["decisions"]["assist_torque_y_nm"], 0.)
            np.testing.assert_array_equal(a["decisions"]["mode"], "UNCERTAIN")
            public = policy_observation({k: v[0] for k, v in a["observations"].items()})
            self.assertFalse(set(public) & {"ground_truth_intent", "human_goal_angle", "human_event_timestamp"})
            with np.load(first / "teacher_inputs.npz") as teacher:
                for key in teacher.files:
                    np.testing.assert_array_equal(teacher[key], p[key])
            with self.assertRaises(FileExistsError):
                run_human(first)
            (first / "events.json").write_text("[]")
            with self.assertRaisesRegex(ValueError, "hash mismatch"):
                run_human(Path(temp) / "tampered", mode="replay", replay=first)

    def test_manual_callback_timing_stop_and_replay_use_real_pipeline(self):
        # Only the OS window is replaced; physics, polling, events and logging are real.
        class Viewer:
            def __init__(self, m, d, key_callback):
                self.callback, self.frames = key_callback, 0
                self.cam = SimpleNamespace(lookat=np.zeros(3))
                self.user_scn = mujoco.MjvScene(m, maxgeom=20)
            def __enter__(self):
                return self
            def __exit__(self, *args):
                pass
            def is_running(self):
                return True
            def lock(self):
                return nullcontext()
            def sync(self):
                self.callback(ord("LKJQ"[self.frames]))
                self.frames += 1
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            with patch("mujoco.viewer.launch_passive", Viewer), patch("phase1b.human.time.sleep"):
                report = run_human(root / "manual", mode="manual", show=True, duration=2.)
            self.assertEqual(report["mode"], "manual")
            self.assertLess(report["duration_seconds"], 2.)
            events = json.loads((root / "manual/events.json").read_text())
            self.assertEqual([e["mode"] for e in events], ["HOLD", "ROTATE_POS", "HOLD", "ROTATE_NEG"])
            self.assertAlmostEqual(events[1]["timestamp"], .042)
            self.assertTrue(all(e["applied_wall_seconds"] >= e["received_wall_seconds"] for e in events[1:]))
            self.assertEqual(report["raw_inputs"][-1]["status"], "stop_requested")
            run_human(root / "replay", mode="replay", replay=root / "manual")
            a, _ = read_episode(root / "manual/episode")
            b, _ = read_episode(root / "replay/episode")
            for section in a:
                for key in a[section]:
                    np.testing.assert_array_equal(a[section][key], b[section][key])
            self.assertFalse(a["supervision"]["expert_label_valid"].any())

    def test_failure_keeps_events_and_partial_evidence_without_teacher_export(self):
        calls = 0

        def fail(world):
            nonlocal calls
            calls += 1
            if calls == 8:
                raise RuntimeError("injected physical guard failure")

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp) / "failed"
            with patch("phase1b.human.physical_guard", fail), self.assertRaisesRegex(RuntimeError, "injected"):
                run_human(root, duration=.1)
            report = json.loads((root / "session.json").read_text())
            self.assertEqual(report["status"], "failed")
            self.assertFalse(report["expert_labels_validated"])
            self.assertTrue((root / "events.json").exists())
            self.assertTrue((root / "episode/partial_observations.npz").exists())
            self.assertFalse((root / "teacher_inputs.npz").exists())
            self.assertFalse((root / "episode/verification.json").exists())


if __name__ == "__main__":
    unittest.main()
