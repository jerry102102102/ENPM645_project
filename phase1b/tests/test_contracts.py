"""Contract, isolation and real-pipeline smoke tests; no Expert/tracking claims."""

import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

from phase1b.control import ControlChain, ScriptedContractPolicy
from phase1b.interfaces import (
    POLICY_SHAPES,
    AssistanceDecision,
    AssistTorqueReference,
    PositionHoldDualArmAdapter,
    ReferenceOnlyObjectAdapter,
    history_view,
    public_input,
)
from phase1b.pipeline import (
    controller_input,
    read_episode,
    run_episode,
    validate_episode,
)
from phase1b.simulation import PandaWorld


def decision(mode="HOLD", torque=0.01, **kw):
    return AssistanceDecision(
        kw.pop("timestamp", 0.0),
        mode,
        torque,
        kw.pop("confidence", None),
        kw.pop("source", "scripted_test"),
        kw.pop("valid", True),
        **kw,
    )


class Contracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.world = PandaWorld()
        cls.world.refresh(0)
        cls.observation, _ = cls.world.snapshot()
        cls.inp = controller_input(cls.world, cls.observation)

    def test_modes_signs_confidence_and_invalid(self):
        for m, t in (
            ("ROTATE_POS", 0.02),
            ("ROTATE_NEG", -0.02),
            ("HOLD", 0.01),
            ("HOLD", -0.01),
            ("UNCERTAIN", 0.0),
        ):
            self.assertEqual(decision(m, t).assist_torque_y_nm, t)
        self.assertFalse(decision("UNCERTAIN", 0.0, valid=False).valid)
        for m, t, kw in (
            ("C", 0.0, {}),
            ("UNCERTAIN", 0.01, {}),
            ("HOLD", 0.0, {"valid": False}),
            ("HOLD", np.nan, {}),
            ("HOLD", True, {}),
            ("HOLD", 0.0, {"valid": 1}),
            ("HOLD", 0.0, {"confidence": np.nan}),
            ("HOLD", 0.0, {"confidence": 1.1}),
            ("HOLD", 0.0, {"timestamp": -0.1}),
            ("HOLD", 0.0, {"source": "A"}),
        ):
            with (
                self.subTest(mode=m, torque=t, kwargs=kw),
                self.assertRaises(ValueError),
            ):
                decision(m, t, **kw)

    def test_legacy_scalar_adapter_requires_explicit_mode(self):
        ref = AssistTorqueReference(0, 0.01)
        self.assertEqual(AssistanceDecision.from_legacy(ref, "HOLD").to_legacy(), ref)
        self.assertEqual(
            decision(source="oracle_expert").to_legacy().source, "model_based_expert"
        )
        with self.assertRaises(ValueError):
            decision(source="rule_based").to_legacy()
        with self.assertRaises(TypeError):
            AssistanceDecision.from_legacy(ref)
        self.assertIsNone(decision("UNCERTAIN", 0.0, valid=False).to_legacy().torque_nm)

    def test_frames_units_quaternion_and_pitch_responsibility(self):
        c = ReferenceOnlyObjectAdapter().step(decision(), self.inp, 0.01)
        np.testing.assert_array_equal(c.assist_wrench_world, [0, 0, 0, 0, 0.01, 0])
        np.testing.assert_array_equal(
            c.pose_ref_world[3:], self.observation["object_orientation"]
        )
        for kw in (
            {"frame_id": "gripper"},
            {"reference_point": "wrist"},
            {"pose_ref_world": np.zeros(7)},
            {"twist_ref_world": np.zeros(7)},
            {"assist_wrench_world": np.array([0.01, 0, 0, 0, 0, 0])},
            {"impedance_parameters": {"stiffness": np.ones(6)}},
            {"impedance_parameters": {"unknown": np.zeros(6)}},
        ):
            with self.subTest(kw=kw), self.assertRaises(ValueError):
                replace(c, **kw)
        stiffness = np.ones(6)
        stiffness[1] = 0
        tuned = replace(c, impedance_parameters={"stiffness": stiffness})
        stiffness[:] = 99
        self.assertEqual(tuned.impedance_parameters["stiffness"][1], 0)
        with self.assertRaises(ValueError):
            tuned.pose_ref_world[0] = 5
        with self.assertRaises(TypeError):
            tuned.impedance_parameters["damping"] = np.zeros(6)
        # Explicit angular->linear order is preserved; no implicit permutation.
        twist = np.array([0.1, 0.2, 0.3, 1.0, 2.0, 3.0])
        np.testing.assert_array_equal(
            replace(c, twist_ref_world=twist).twist_ref_world, twist
        )

    def test_causal_isolation_and_joint_opt_in(self):
        contaminated = dict(
            self.observation,
            human_goal=4,
            scenario_label="C",
            human_applied_force=np.ones(3),
            external_disturbance_force=np.ones(3),
            future_observation={},
            future_command={},
        )
        clean = public_input(replace(self.inp, observation=contaminated))
        for key in set(contaminated) - set(self.observation):
            self.assertNotIn(key, clean.observation)
        h = history_view([contaminated], 0)
        self.assertEqual(set(h.features), set(POLICY_SHAPES))
        self.assertNotIn("arm_joint_position", h.features)
        self.assertIn(
            "arm_joint_position",
            history_view([contaminated], 0, "with_joint_states").features,
        )
        for samples in (
            [dict(contaminated, timestamp=0.01)],
            [contaminated, contaminated],
        ):
            with self.assertRaises(ValueError):
                history_view(samples, 0)
        with self.assertRaises(ValueError):
            public_input(
                replace(self.inp, observation=dict(contaminated, timestamp=0.01))
            )
        with self.assertRaises(ValueError):
            h.features["object_position"][0, 0] = 9

    def test_hold_latch_uncertain_and_substep_clock(self):
        adapter = ReferenceOnlyObjectAdapter()
        first = adapter.step(decision(), self.inp, 0.01)
        observation = dict(
            self.observation,
            timestamp=0.01,
            object_position=self.observation["object_position"] + [0.01, 0, 0],
        )
        inp = replace(self.inp, timestamp=0.01, observation=observation)
        held = adapter.step(decision(timestamp=0.01, torque=-0.01), inp, 0.01)
        np.testing.assert_array_equal(first.pose_ref_world, held.pose_ref_world)
        uncertain = adapter.step(
            decision("UNCERTAIN", 0.0, timestamp=0.01, valid=False), inp, 0.01
        )
        np.testing.assert_array_equal(uncertain.twist_ref_world, 0)
        self.assertFalse(
            PositionHoldDualArmAdapter().step(uncertain, inp).assist_reference_executed
        )
        with self.assertRaises(ValueError):
            PositionHoldDualArmAdapter().step(held, self.inp)
        with self.assertRaises(ValueError):
            PositionHoldDualArmAdapter().step(first, inp)

    def test_chain_rejects_future_unbounded_and_mutated_intent(self):
        class FixedPolicy:
            def __init__(self, d):
                self.d = d

            def predict(self, h):
                return self.d

        for d in (decision(timestamp=0.01), decision(torque=0.2)):
            with self.assertRaises(ValueError):
                ControlChain(policy=FixedPolicy(d)).step(self.inp)

        class WrongObject:
            def step(self, d, inp, dt):
                c = ReferenceOnlyObjectAdapter().step(d, inp, dt)
                return replace(c, assist_wrench_world=np.zeros(6))

        with self.assertRaises(ValueError):
            ControlChain(object_controller=WrongObject()).step(self.inp)

        class FastObject:
            def step(self, d, inp, dt):
                return replace(
                    ReferenceOnlyObjectAdapter().step(d, inp, dt),
                    twist_ref_world=np.array([0.0, 2.0, 0.0, 0.0, 0.0, 0.0]),
                )

        with self.assertRaises(ValueError):
            ControlChain(object_controller=FastObject()).step(self.inp)

    def test_three_layer_logging_and_no_automatic_expert_labels(self):
        class OracleNamedPolicy(ScriptedContractPolicy):
            def predict(self, h):
                return replace(super().predict(h), source="oracle_expert")

        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "episode"
            result = run_episode(p, "C", 0.12, policy=OracleNamedPolicy())
            self.assertTrue(result["interface_ready"])
            self.assertTrue(result["physical_sample_checks_passed"])
            d, m = read_episode(p)
            self.assertEqual(
                set(d["decisions"]["mode"]),
                {"ROTATE_POS", "ROTATE_NEG", "HOLD", "UNCERTAIN"},
            )
            self.assertFalse(d["decisions"]["confidence_present"].any())
            self.assertFalse(d["supervision"]["expert_label_valid"].any())
            self.assertFalse(d["controller"]["assist_reference_executed"].any())
            np.testing.assert_array_equal(
                d["controller"]["decision_timestamp"],
                np.repeat(d["observations"]["timestamp"][:, None], 5, axis=1),
            )
            self.assertTrue(
                np.isnan(d["controller"]["allocated_gripper_wrenches"]).all()
            )
            # A changed reference time must fail readback, not silently become a valid record.
            d["controller"]["object_command_timestamp"][0, 2] += 0.01
            np.savez_compressed(p / "controller.npz", **d["controller"])
            with self.assertRaises(AssertionError):
                validate_episode(p)

    def test_adapters_leave_physics_unchanged(self):
        class UncertainPolicy:
            def predict(self, h):
                return decision("UNCERTAIN", 0.0, timestamp=float(h.timestamps[-1]))

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            run_episode(root / "requested", "C", 0.12)
            run_episode(root / "uncertain", "C", 0.12, policy=UncertainPolicy())
            a, _ = read_episode(root / "requested")
            b, _ = read_episode(root / "uncertain")
            for section in (
                "observations",
                "next_observations",
                "privileged",
                "render_states",
                "diagnostics",
            ):
                for key in a[section]:
                    np.testing.assert_array_equal(a[section][key], b[section][key])


if __name__ == "__main__":
    unittest.main()
