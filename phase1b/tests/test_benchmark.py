"""Fixture/input tests; successful transport must never imply torque tracking."""

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from phase1b.benchmark import BenchmarkWorld, InputSpec, TorqueInput, physical_guard, run_benchmark
from phase1b.control import ControlChain
from phase1b.interfaces import AssistanceDecision, PUBLIC_SHAPES, PositionHoldDualArmAdapter, ReferenceOnlyObjectAdapter
from phase1b.pipeline import controller_input, read_episode


class FailAfterTransition:
    def step(self, command, observation):
        if observation.timestamp >= 0.01:
            raise RuntimeError("intentional test failure")
        return PositionHoldDualArmAdapter().step(command, observation)


class ObservableTestPolicy:
    def predict(self, history):
        assert "human_goal" not in history.features
        assert "scenario_label" not in history.features
        assert "external_disturbance_force" not in history.features
        return AssistanceDecision(float(history.timestamps[-1]), "HOLD", .001,
                                  None, "scripted_test", True)


class BenchmarkTests(unittest.TestCase):
    def test_waveform_sign_plateaus_endpoints_slew_and_invalid_inputs(self):
        spec = InputSpec()
        t = np.arange(0, 7.001, 0.002)
        a = np.array([spec.torque("T1", x) for x in t])
        b = np.array([spec.torque("T2", x) for x in t])
        np.testing.assert_array_equal(b, -a)
        self.assertEqual(spec.torque("T0", 2), 0)
        self.assertEqual(spec.torque("T1", 1), 0)
        self.assertEqual(spec.torque("T1", 2), 0.01)
        self.assertEqual(spec.torque("T1", 3), 0)
        self.assertEqual(spec.torque("T3", 5), -0.01)
        self.assertEqual(spec.torque("T3", 7), 0)
        self.assertLessEqual(np.max(np.abs(np.diff(a) / .002)), spec.max_slew_nm_s)
        for kwargs in ({"amplitude_nm": .03}, {"ramp_seconds": .1},
                       {"onset_seconds": .001}, {"zero_seconds": 0},
                       {"hold_seconds": np.nan}, {"amplitude_nm": True}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                InputSpec(**kwargs)
        with self.assertRaises(ValueError):
            spec.duration("T9")

    def test_checkpoint_replay_fixed_human_impedance_and_public_callbacks(self):
        world = BenchmarkWorld()
        saved = world.prepare()
        world.restore(saved)
        first, _ = world.snapshot()
        trajectory = []
        for _ in range(10):
            world.step()
            trajectory.append(world.d.qpos.copy())
        world.restore(saved)
        again, _ = world.snapshot()
        for key in first:
            np.testing.assert_array_equal(first[key], again[key])
        for expected in trajectory:
            world.step()
            np.testing.assert_array_equal(world.d.qpos, expected)
            np.testing.assert_array_equal(world.truth["human_target_trajectory"], saved["fixed_target"])
            np.testing.assert_array_equal(world.truth["external_disturbance_force"], 0)
        world.restore(saved)
        force = world.truth["human_applied_force"].copy()
        world.d.qpos[world.m.jnt_qposadr[world.m.joint("panel_free").id]] += .0001
        world.refresh(0)
        self.assertGreater(np.linalg.norm(world.truth["human_applied_force"] - force), 0)
        np.testing.assert_array_equal(world.truth["human_target_trajectory"], saved["fixed_target"])
        np.testing.assert_array_equal(world.d.qfrc_applied[world.va], 0)

        class PublicObject(ReferenceOnlyObjectAdapter):
            def step(self, decision, inp, dt):
                self.assert_public(inp)
                return super().step(decision, inp, dt)

            @staticmethod
            def assert_public(inp):
                assert set(inp.observation) == set(PUBLIC_SHAPES) | {"timestamp"}
                assert "fixed_target" not in inp.observation
                assert "case" not in inp.observation

        world.restore(saved)
        chain = ControlChain(policy=TorqueInput("T1"), object_controller=PublicObject())
        chain.step(controller_input(world, world.snapshot()[0]))
        self.assertEqual(chain.decision.mode, "HOLD")
        self.assertEqual(chain.decision.source, "scripted_test")
        self.assertIsNone(chain.decision.confidence)

    def test_pipeline_alignment_baseline_comparison_and_honest_verdict(self):
        # Short clock-aligned fixture tests; the full development run uses defaults.
        spec = InputSpec(onset_seconds=.02, ramp_seconds=.02, hold_seconds=.02,
                         zero_seconds=.02, amplitude_nm=.0001, human_angle_deg=.1,
                         correction_angle_deg=.02, disturbance_force_n=.01)
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp) / "run"
            report = run_benchmark(root, spec, cases=tuple(f"T{i}" for i in range(9)))
            self.assertEqual(report["status"], "completed")
            self.assertEqual(len(report["runs"]), 10)
            for name, result in report["runs"].items():
                self.assertTrue(result["verification"]["data_pipeline_passed"])
                self.assertTrue(result["verification"]["physical_sample_checks_passed"])
                self.assertFalse(result["metrics"]["assistance_tracking_validated"])
                data, meta = read_episode(root / name)
                if name != "T7":
                    np.testing.assert_array_equal(data["privileged"]["external_disturbance_force"], 0)
                if name in ("T0", "T1", "T2", "T3", "T0_for_T3", "T7"):
                    np.testing.assert_array_equal(data["privileged"]["human_target_trajectory"],
                                                 np.tile(report["fixed_human_target_world_m"], (len(data["decisions"]["timestamp"]), 1)))
                np.testing.assert_array_equal(data["decisions"]["mode"], "HOLD")
                self.assertFalse(data["supervision"]["expert_label_valid"].any())
                self.assertGreater(meta["engine_time_origin"], 2.0)
                self.assertEqual(meta["experiment"]["run_id"], name)
                np.testing.assert_array_equal(data["privileged"]["scenario_label"], meta["experiment"]["task_id"])
                self.assertEqual(meta["experiment"]["initial_state_sha256"], report["initial_state_sha256"])
            for case in ("T1", "T2", "T3"):
                with np.load(root / f"{case}_comparison.npz") as comparison:
                    np.testing.assert_array_equal(comparison["total_contact_ty_difference_nm"], 0)
                    self.assertFalse(comparison["tracking_valid"].any())
            with self.assertRaises(FileExistsError):
                run_benchmark(root, spec)

    def test_failures_keep_partial_data_and_never_produce_success(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp) / "run"
            with self.assertRaisesRegex(RuntimeError, "intentional test failure"):
                run_benchmark(root, cases=("T0",),
                              dual_arm_controller=__name__ + ":FailAfterTransition")
            report = json.loads((root / "report.json").read_text())
            self.assertEqual(report["status"], "failed")
            self.assertTrue((root / "failure_state.npz").exists())
            failure = json.loads((root / "T0/failure.json").read_text())
            self.assertEqual(failure["completed_transitions"], 1)
            self.assertFalse(failure["partial_data_valid"])
            self.assertTrue((root / "T0/partial_observations.npz").exists())
            self.assertFalse((root / "T0/verification.json").exists())

    def test_human_rotation_correction_disturbance_pause_and_reverse_inputs(self):
        world = BenchmarkWorld()
        saved = world.prepare()
        initial = world.p["initial_angle"]
        for case, angle in (("T4", 10), ("T5", -10), ("T6", 2), ("T8", 5)):
            world.case = case
            world.restore(saved)
            goal, target, velocity, force = world.environment_targets(2, "B")
            self.assertAlmostEqual(goal - initial, np.deg2rad(angle))
            self.assertGreater(np.linalg.norm(target - saved["fixed_target"]), 0)
            np.testing.assert_array_equal(velocity, 0)
            np.testing.assert_array_equal(force, 0)
        world.case = "T7"
        _, target, velocity, force = world.environment_targets(3.9, "B")
        np.testing.assert_array_equal(target, saved["fixed_target"])
        np.testing.assert_array_equal(force, [0, 0, -2])
        world.case = "T8"
        self.assertAlmostEqual(world.environment_targets(3, "B")[0] - initial, np.deg2rad(10))
        world.reverse_after_pause = True
        np.testing.assert_array_equal(world.environment_targets(3, "B")[1], saved["fixed_target"])
        world.case = "T6"
        np.testing.assert_array_equal(world.environment_targets(3, "B")[1], saved["fixed_target"])
        world.case = "T4"
        world.spec = InputSpec(human_angle_deg=100)
        with self.assertRaisesRegex(ValueError, "speed limit"):
            world.environment_targets(1.25, "B")

    def test_cooperation_policy_injection_and_matched_zero_baseline(self):
        spec = InputSpec(onset_seconds=.02, ramp_seconds=.02, hold_seconds=.02,
                         zero_seconds=.02, amplitude_nm=.0001, human_angle_deg=.1)
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp) / "run"
            report = run_benchmark(root, spec, cases=("T4",),
                                   policy=__name__ + ":ObservableTestPolicy")
            self.assertEqual(set(report["runs"]), {"T4", "T4_zero"})
            a, _ = read_episode(root / "T4")
            b, _ = read_episode(root / "T4_zero")
            np.testing.assert_array_equal(a["decisions"]["assist_torque_y_nm"], .001)
            np.testing.assert_array_equal(b["decisions"]["assist_torque_y_nm"], 0)
            np.testing.assert_array_equal(a["privileged"]["human_target_trajectory"],
                                         b["privileged"]["human_target_trajectory"])
            self.assertFalse(a["supervision"]["expert_label_valid"].any())
            self.assertFalse(report["runs"]["T4"]["metrics"]["assistance_tracking_validated"])

    def test_physical_guard_rejects_contact_loss(self):
        world = BenchmarkWorld()
        world.prepare()
        physical_guard(world)
        for gid in sum(world.pad_g, []):
            world.m.geom_contype[gid] = 0
            world.m.geom_conaffinity[gid] = 0
        world.refresh(0)
        with self.assertRaisesRegex(RuntimeError, "lost pad contact"):
            physical_guard(world)


if __name__ == "__main__":
    unittest.main()
