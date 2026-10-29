"""Contract failures, frame signs, causal isolation, and controller injection."""

import importlib
import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

from phase0_tactile.scripts.run_experiment import rotate_y
from phase0_v2.simulation.dual_gripper import point_wrench
from phase1a.data.episode_io import DatasetReader, validate_dataset
from phase1a.data.observation_builder import (
    POLICY_SHAPES,
    policy_observation_builder,
)
from phase1a.interfaces.controller_contract import (
    AssistTorqueReference,
    SupportOnlyAdapter,
    validate_input,
    validate_output,
)
from phase1a.scripts.run_pipeline_smoke import run_episode, scripted_reference
from phase1a.simulation.scenario_loader import CONFIG, load_scenario
from phase1a.simulation.transition_world import TransitionWorld


class DelegatingController:
    """Test-only alternate entry point: delegates support without adding allocation."""

    def step(self, reference, observation):
        return SupportOnlyAdapter().step(reference, observation)


class Contracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_scenario()
        cls.world = TransitionWorld(cls.config, "B")
        cls.obs = cls.world.current
        cls.inp = cls.world.controller_input()
        cls.ref = AssistTorqueReference(0.0, 0.02)
        cls.out = SupportOnlyAdapter().step(cls.ref, cls.inp)

    def test_frozen_geometry_and_no_world_fixture(self):
        scene = self.config["scene"]
        self.assertEqual(
            scene["grippers"]["left"]["nominal_grasp_position_panel_local"],
            [-0.2, 0.0, 0.0],
        )
        self.assertEqual(
            scene["grippers"]["right"]["nominal_grasp_position_panel_local"],
            [0.2, 0.0, 0.0],
        )
        self.assertEqual(scene["human_grasp_point_panel_local"], [0.27, -0.08, 0.0])
        self.assertEqual(scene["equality_constraints"], 0)
        for side in ("left", "right"):
            self.assertEqual(scene["grippers"][side]["root_joint_type"], 0)

    def test_mutated_config_rejected(self):
        with tempfile.TemporaryDirectory() as t:
            changed = json.loads(CONFIG.read_text())
            changed["parameters"]["friction"] += 0.01
            p = Path(t) / "changed.json"
            p.write_text(json.dumps(changed))
            with self.assertRaises(ValueError):
                load_scenario(p)

    def test_reference_sign_and_invalid_values(self):
        self.assertLess((rotate_y(0.01) @ np.array([1.0, 0, 0]))[2], 0)
        self.assertGreater(
            point_wrench(np.array([0.0, 0, -1.0]), np.array([0.2, 0, 0]), np.zeros(3))[
                4
            ],
            0,
        )
        self.assertEqual(self.ref.assistance_wrench()[4], 0.02)
        for kw in (
            {"torque_nm": np.nan},
            {"torque_nm": None},
            {"frame_id": "panel"},
            {"source": "expert"},
            {"reference_point": "world_origin"},
            {"timestamp": -1},
            {"valid": False},
        ):
            with self.subTest(kw=kw), self.assertRaises(ValueError):
                replace(self.ref, **kw)
        self.assertIsNone(
            replace(self.ref, valid=False, torque_nm=None).assistance_wrench()
        )

    def test_scripted_schedule(self):
        refs = [
            scripted_reference(t, self.config, True).torque_nm
            for t in (0.0, 1.0, 2.0, 3.0, 4.0)
        ]
        self.assertEqual(refs, [0.0, 0.02, 0.0, -0.02, 0.0])

    def test_support_ignores_positive_and_negative_reference(self):
        for tau in (-0.02, 0.0, 0.02):
            out = SupportOnlyAdapter().step(replace(self.ref, torque_nm=tau), self.inp)
            np.testing.assert_array_equal(
                out.executed_virtual_drive_wrenches, self.inp.baseline_drive_proposal
            )
            self.assertFalse(out.assist_reference_executed)
            self.assertIsNone(out.allocated_gripper_wrenches)
            self.assertIsNone(out.requested_total_object_wrench)

    def test_input_blocks_privileged_and_future_values(self):
        with self.assertRaises(ValueError):
            validate_input(replace(self.ref, timestamp=0.002), self.inp)
        sensor = {**self.inp.sensor, "human_goal": np.array([0.0])}
        with self.assertRaises(ValueError):
            validate_input(self.ref, replace(self.inp, sensor=sensor))
        sensor = {**self.inp.sensor, "tactile_left": np.full((2, 3, 4, 4), np.nan)}
        with self.assertRaises(ValueError):
            validate_input(self.ref, replace(self.inp, sensor=sensor))

    def test_output_rejects_wrong_shape_nan_limits_and_fake_feedback(self):
        for kw in (
            {"executed_virtual_drive_wrenches": np.zeros(6)},
            {"executed_virtual_drive_wrenches": np.full((2, 6), np.nan)},
            {"executed_virtual_drive_wrenches": np.full((2, 6), 100.0)},
            {"executed_actuator_commands": np.full(4, 100.0)},
            {"saturation_status": np.zeros((2, 2))},
            {"contact_status": np.zeros((2, 2))},
            {"assist_reference_executed": True},
            {"realized_object_wrench": np.zeros(6)},
        ):
            with self.subTest(kw=list(kw)), self.assertRaises(ValueError):
                validate_output(replace(self.out, **kw), self.inp)

    def test_history_rejects_future_duplicate_and_hidden_fields(self):
        later = replace(
            self.obs,
            timestamp=0.01,
            physics_tick=5,
            engine_timestamp=self.obs.engine_timestamp + 0.01,
        )
        for frames in ([self.obs, later], [self.obs, self.obs]):
            with self.assertRaises(ValueError):
                policy_observation_builder(frames, 0.0)
        with self.assertRaises(ValueError):
            replace(self.obs, fields={**self.obs.fields, "human_goal": np.array([1.0])})
        history = policy_observation_builder([self.obs], 0.0)
        self.assertEqual(set(history.features), set(POLICY_SHAPES))
        with self.assertRaises(ValueError):
            history.features["wrist_ft_left"][0, 0] = 999

    def test_new_controller_import_and_dataset_poisoning(self):
        # Exercise the same module:Class mechanism used by --controller.
        factory = getattr(importlib.import_module(__name__), "DelegatingController")
        with tempfile.TemporaryDirectory() as t:
            path = Path(t) / "plugin"
            run_episode(
                path, "B", self.config, duration=0.03, controller_factory=factory
            )
            self.assertTrue(validate_dataset(path)["passed"])
            reader = DatasetReader(path)
            before = reader.policy_history(0)
            # Future sensors, post-action sensors, supervision and hidden truth must not enter o_0.
            reader.sections["observations"]["wrist_ft_left"][1:] = 1e9
            for sec in ("next_observations", "privileged", "supervision", "controller"):
                reader.sections[sec] = {
                    k: np.full_like(v, 999) if v.dtype.kind in "fi" else v
                    for k, v in reader.sections[sec].items()
                }
            after = reader.policy_history(0)
            for key in POLICY_SHAPES:
                np.testing.assert_array_equal(before.features[key], after.features[key])
            # A corrupt event clock must be rejected on disk.
            with np.load(path / "controller.npz") as f:
                ctrl = {k: f[k] for k in f.files}
            ctrl["decision_timestamp"][0] = 0.02
            np.savez_compressed(path / "controller.npz", **ctrl)
            with self.assertRaises(AssertionError):
                validate_dataset(path)

    def test_valid_expert_and_fake_zero_unavailable_rejected(self):
        with tempfile.TemporaryDirectory() as t:
            path = Path(t) / "test"
            run_episode(path, "B", self.config, duration=0.01)
            with np.load(path / "supervision.npz") as f:
                expert = {k: f[k] for k in f.files}
            expert["expert_label_valid"][0] = True
            np.savez_compressed(path / "supervision.npz", **expert)
            with self.assertRaises(AssertionError):
                validate_dataset(path)
            expert["expert_label_valid"][0] = False
            np.savez_compressed(path / "supervision.npz", **expert)
            with np.load(path / "controller.npz") as f:
                ctrl = {k: f[k] for k in f.files}
            ctrl["tracking_error_Nm"][0] = 0.0
            np.savez_compressed(path / "controller.npz", **ctrl)
            with self.assertRaises(AssertionError):
                validate_dataset(path)


if __name__ == "__main__":
    unittest.main()
