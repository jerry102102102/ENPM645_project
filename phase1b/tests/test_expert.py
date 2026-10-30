"""Hand-checkable fixed-input tests only: no MuJoCo imports or physics stepping."""

import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

from phase1b.expert.data import (
    ROOT,
    annotate_episode,
    digest,
    generate_candidates,
    load_arrays,
    validate_sidecar,
)
from phase1b.expert.learnability import assess, windows
from phase1b.expert.reference import (
    DisturbanceTruth,
    ExpertConfig,
    PublicSample,
    ReferenceGenerator,
    TeacherGoal,
    human_pitch_impedance,
    panel_inertia,
    required_assistance_y,
)
from phase1b.interfaces import AssistanceDecision, ObjectCommand


def config(**changes):
    # Analytic test constants, NOT reachable/safe operating limits for the robot.
    c = ExpertConfig(
        inertia_y=0.03,
        force_gain=2.0,
        lpf_time_constant=0.001,
        kp_rotate=2.0,
        kd_rotate=0.1,
        kp_hold=3.0,
        kd_hold=0.2,
        torque_max=1.0,
        torque_rate_max=1000.0,
        reference_rate_max=0.1,
        gate_torque_on=0.2,
        gate_torque_off=0.1,
        gate_omega_on=0.0,
        gate_omega_off=0.0,
        gate_confirm_seconds=0.02,
        gate_release_seconds=0.02,
        disturbance_fraction=0.5,
        limits_provenance="unit-test arithmetic only; not hardware validated",
    )
    return replace(c, **changes)


def sample(t, theta=0.0, omega=0.0, robot=-0.4):
    return PublicSample(t, theta, omega, robot / 2, robot / 2)


def fixtures(n=6):
    t = np.arange(n) * 0.01
    obs = dict(
        timestamp=t,
        object_position=np.zeros((n, 3)),
        object_orientation=np.tile([1.0, 0, 0, 0], (n, 1)),
        object_linear_velocity=np.zeros((n, 3)),
        object_angular_velocity=np.zeros((n, 3)),
        end_effector_pose=np.tile(
            [[-0.2, 0.1, 0, 1, 0, 0, 0], [0.2, 0.1, 0, 1, 0, 0, 0]], (n, 1, 1)
        ),
        arm_joint_position=np.zeros((n, 2, 7)),
        arm_joint_velocity=np.zeros((n, 2, 7)),
    )
    for s in ("left", "right"):
        obs["wrist_ft_" + s] = np.zeros((n, 6))
        obs["wrist_sensor_" + s] = np.zeros((n, 6))
        obs["tactile_" + s] = np.zeros((n, 2, 3, 4, 4))
    teacher = dict(
        timestamp=t,
        human_goal_angle=np.zeros(n),
        ground_truth_intent=np.full(n, "HOLD"),
        external_disturbance_wrench=np.zeros((n, 6)),
    )
    return obs, teacher


class ExpertMathTests(unittest.TestCase):
    def test_rigid_body_and_hand_math(self):
        inertia = panel_inertia(1.0, [0.6, 0.2, 0.02])
        self.assertAlmostEqual(inertia[1, 1], 0.0300333333333333)
        self.assertAlmostEqual(
            required_assistance_y(inertia, [0, 0, 0], [0, 2, 0], 0.01, 0.02, 0.03),
            2 * inertia[1, 1] - 0.06,
        )
        # Nonprincipal-axis rotation includes gyro coupling, not just Jy*alpha_y.
        self.assertAlmostEqual(
            required_assistance_y(
                np.diag([1.0, 2.0, 3.0]), [1, 2, 3], [0, 0, 0], 0, 0, 0
            ),
            -6.0,
        )
        kh, dh = human_pitch_impedance(100.0, 10.0, [0.27, -0.1, 0])
        self.assertAlmostEqual(kh, 7.29)
        self.assertAlmostEqual(dh, 0.729)
        for kw in (
            {"torque_max": 0.0},
            {"torque_rate_max": np.nan},
            {"disturbance_fraction": 1.1},
            {"limits_provenance": ""},
        ):
            with self.assertRaises(ValueError):
                config(**kw)

    def test_public_ft_frame_and_privileged_isolation(self):
        obs, _ = fixtures(1)
        row = {k: v[0] for k, v in obs.items()}
        row["wrist_ft_left"][2] = -2.0
        first = PublicSample.from_observation(row)
        self.assertAlmostEqual(first.robot_torque_left_y, 0.4)
        row.update(
            human_goal=100, external_disturbance_wrench=np.ones(6), scenario_label="C"
        )
        self.assertEqual(first, PublicSample.from_observation(row))
        e0 = ReferenceGenerator("E0", config())
        with self.assertRaises(ValueError):
            e0.step(first, goal=TeacherGoal(0, "HOLD", 0))
        with self.assertRaises(ValueError):
            ReferenceGenerator("E1", config()).step(
                first,
                goal=TeacherGoal(0, "HOLD", 0),
                disturbance=DisturbanceTruth(0, 1),
            )
        with self.assertRaises(ValueError):
            ReferenceGenerator("E1", config()).step(
                first, goal=TeacherGoal(0.01, "HOLD", 0)
            )

    def test_e0_causal_derivative_and_lpf(self):
        g = ReferenceGenerator("E0", config(lpf_time_constant=0.01))
        self.assertFalse(g.step(sample(0, robot=-0.1)).candidate_valid)
        a = g.step(sample(0.01, robot=-0.1))
        self.assertAlmostEqual(a.acceleration_estimate, 0)
        self.assertAlmostEqual(a.interaction_torque_estimate, 0.1)
        self.assertAlmostEqual(a.decision.assist_torque_y_nm, 0.2)
        b = g.step(sample(0.02, robot=-0.2))
        self.assertAlmostEqual(
            b.filtered_interaction_torque, 0.1 + (1 - np.exp(-1)) * 0.1
        )
        with self.assertRaises(ValueError):
            g.step(sample(0.02))
        with self.assertRaises(ValueError):
            g.step(sample(0.04))

    def test_e1_observable_gate_reference_rate_and_no_mindreading(self):
        g = ReferenceGenerator("E1", config())
        for t in (0.0, 0.01):
            r = g.step(sample(t), goal=TeacherGoal(t, "ROTATE_POS", 0.5))
            self.assertEqual(r.decision.mode, "UNCERTAIN")
            self.assertEqual(r.reference_angle, 0)
        r = g.step(sample(0.02), goal=TeacherGoal(0.02, "ROTATE_POS", 0.5))
        self.assertTrue(r.intent_gate)
        self.assertAlmostEqual(r.reference_angle, 0.001)
        self.assertAlmostEqual(r.reference_velocity, 0.1)
        self.assertAlmostEqual(r.torque_before_limit, 0.03 * 10 + 2 * 0.001 + 0.1 * 0.1)
        # Clear opposite physical cue cancels even while angular velocity remains positive.
        r = g.step(
            sample(0.03, omega=0.0, robot=0.4),
            goal=TeacherGoal(0.03, "ROTATE_POS", 0.5),
        )
        self.assertFalse(r.intent_gate)
        self.assertEqual(r.decision.assist_torque_y_nm, 0)
        cold = ReferenceGenerator("E1", config())
        for t in (0.0, 0.01, 0.02, 0.03):
            self.assertEqual(
                cold.step(
                    sample(t, robot=0), goal=TeacherGoal(t, "ROTATE_POS", 1.0)
                ).decision.assist_torque_y_nm,
                0,
            )

    def test_braking_sign_hold_latch_and_gate_release(self):
        self.assertEqual(
            AssistanceDecision(
                0, "ROTATE_NEG", 0.1, None, "oracle_expert", True
            ).assist_torque_y_nm,
            0.1,
        )
        negative = ReferenceGenerator("E1", config())
        for t in (0.0, 0.01, 0.02):
            result = negative.step(
                sample(t, robot=0.4), goal=TeacherGoal(t, "ROTATE_NEG", -0.2)
            )
        self.assertTrue(result.intent_gate)
        self.assertLess(result.decision.assist_torque_y_nm, 0)

        g = ReferenceGenerator("E1", config())
        for t in (0.0, 0.01, 0.02, 0.03):
            r = g.step(sample(t, omega=1.0), goal=TeacherGoal(t, "ROTATE_POS", 0.5))
        self.assertEqual(r.decision.mode, "ROTATE_POS")
        self.assertLess(r.decision.assist_torque_y_nm, 0)
        ObjectCommand(
            0.03,
            "ROTATE_POS",
            np.array([0.0, 0, 0, 1, 0, 0, 0]),
            np.zeros(6),
            np.array([0.0, 0, 0, 0, r.decision.assist_torque_y_nm, 0]),
        )
        hold = ReferenceGenerator("E1", config())
        hold.step(sample(0, theta=0.2), goal=TeacherGoal(0, "HOLD", 9.0))
        r = hold.step(
            sample(0.01, theta=0.3, omega=0.2), goal=TeacherGoal(0.01, "HOLD", 9.0)
        )
        self.assertAlmostEqual(r.reference_angle, 0.2)
        self.assertAlmostEqual(r.torque_before_limit, 3 * (-0.1) - 0.2 * 0.2)
        stopped = ReferenceGenerator("E1", config())
        for t in (0.0, 0.01, 0.02):
            stopped.step(sample(t), goal=TeacherGoal(t, "ROTATE_POS", 0.5))
        for t in (0.03, 0.04):
            r = stopped.step(
                sample(t, robot=0.0), goal=TeacherGoal(t, "ROTATE_POS", 0.5)
            )
        self.assertEqual(r.decision.mode, "UNCERTAIN")

    def test_e2_lambda_and_uncertain_precedence(self):
        for lam in (0.0, 0.5, 1.0):
            g = ReferenceGenerator("E2", config(disturbance_fraction=lam))
            r = g.step(
                sample(0),
                goal=TeacherGoal(0, "HOLD", 0),
                disturbance=DisturbanceTruth(0, 0.4),
            )
            self.assertAlmostEqual(r.torque_before_limit, -lam * 0.4)
            self.assertFalse(r.execution_valid)
            self.assertFalse(r.supervision_eligible)
            r = g.step(
                sample(0.01),
                goal=TeacherGoal(0.01, "UNCERTAIN", 0),
                disturbance=DisturbanceTruth(0.01, 10.0),
            )
            self.assertEqual(r.decision.assist_torque_y_nm, 0)
        with self.assertRaises(TypeError):
            ReferenceGenerator("E2", config()).step(
                sample(0), goal=TeacherGoal(0, "HOLD", 0)
            )

    def test_amplitude_slew_and_explicit_safety_cancellation(self):
        g = ReferenceGenerator("E2", config(torque_max=0.1, torque_rate_max=1.0))
        for t in (0.0, 0.01, 0.02):
            r = g.step(
                sample(t),
                goal=TeacherGoal(t, "HOLD", 0),
                disturbance=DisturbanceTruth(t, 2.0),
            )
            self.assertTrue(r.torque_saturated)
            self.assertTrue(r.torque_rate_limited)
        self.assertAlmostEqual(r.decision.assist_torque_y_nm, -0.03)
        r = g.step(
            sample(0.03),
            goal=TeacherGoal(0.03, "UNCERTAIN", 0),
            disturbance=DisturbanceTruth(0.03, 2.0),
        )
        self.assertTrue(r.rate_limit_overridden)
        self.assertEqual(r.decision.assist_torque_y_nm, 0)
        with self.assertRaises(ValueError):
            AssistanceDecision(0, "UNCERTAIN", 0.01, None, "oracle_expert", True)

    def test_shadow_sidecar_metadata_and_invalid_labels(self):
        obs, teacher = fixtures()
        arrays, derived = generate_candidates(obs, teacher, config())
        self.assertFalse(arrays["E0"]["candidate_valid"][0])
        self.assertFalse(derived["acceleration_valid"][0])
        missing = dict(teacher)
        missing.pop("ground_truth_intent")
        with self.assertRaises(ValueError):
            generate_candidates(obs, missing, config())
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / "source"
            source.mkdir()
            np.savez_compressed(source / "observations.npz", **obs)
            for name in ("privileged", "controller", "object_commands"):
                np.savez_compressed(
                    source / (name + ".npz"), timestamp=obs["timestamp"]
                )
            teacher_path = Path(tmp) / "teacher.npz"
            np.savez_compressed(teacher_path, **teacher)
            meta = dict(
                model_hash=digest(ROOT / "phase1b/models/dual_panda_v1.xml"),
                config_hash=digest(ROOT / "phase1b/config/dual_panda_v1.json"),
                hardware_hash=digest(ROOT / "phase1b/config/hardware_v2.json"),
                schema_version="analytic_fixture",
                physics_dt=0.002,
                duration_seconds=0.06,
            )
            (source / "metadata.json").write_text(json.dumps(meta))
            output = Path(tmp) / "shadow"
            annotate_episode(
                source,
                output,
                config(),
                teacher_inputs_path=teacher_path,
                episode_id="arithmetic-fixture",
            )
            self.assertTrue(validate_sidecar(output))
            metadata = json.loads((output / "metadata.json").read_text())
            self.assertIsNone(metadata["quality"]["episode_valid"])
            for key in ("E0", "E1", "E2"):
                labels = load_arrays(output / "supervision" / f"{key}.npz")
                self.assertFalse(labels["supervision_eligible"].any())
                self.assertTrue(np.isnan(labels["expert_assist_torque_ref"]).all())
            with self.assertRaises(FileExistsError):
                annotate_episode(
                    source,
                    output,
                    config(),
                    teacher_inputs_path=teacher_path,
                    episode_id="same",
                )
            labels["expert_label_valid"][0] = True
            np.savez_compressed(output / "supervision/E2.npz", **labels)
            with self.assertRaises(ValueError):
                validate_sidecar(output)

    def test_causal_windows_profiles_and_neighborhood_diagnostics(self):
        obs, teacher = fixtures()
        labels, _ = generate_candidates(obs, teacher, config())

        def episode(group, torque):
            trace = {k: v.copy() for k, v in labels["E1"].items()}
            trace["candidate_assist_torque_ref"][:] = torque
            return group, obs, trace

        reference = windows(
            [episode("r1", -0.2), episode("r2", 0.2)],
            profile="ft_object",
            history_length=2,
            stride=2,
        )
        query = windows(
            [episode("q", 0.2)], profile="ft_object", history_length=2, stride=2
        )
        report = assess(
            reference, query, radius=0.1, torque_tolerance_nm=0.05, min_neighbors=2
        )
        self.assertEqual(report["coverage"], 1)
        self.assertGreater(report["mean_local_label_std_nm"], 0)
        self.assertEqual(report["inconsistency_fraction"], 1)
        bad = dict(query, labels=np.zeros(1))
        with self.assertRaises(ValueError):
            assess(reference, bad, radius=0.1, torque_tolerance_nm=0.05)
        same = windows([episode("same", 0.0)], profile="ft_object", history_length=2)
        report = assess(
            same, same, radius=0.1, torque_tolerance_nm=0.05, min_neighbors=2
        )
        self.assertEqual(report["status"], "insufficient_matched_evidence")
        self.assertIsNone(report["mean_local_label_std_nm"])
        # Changing future observations cannot change the first window.
        before = query["features"][0].copy()
        obs["object_position"][3:] = 100.0
        after = windows(
            [episode("q", 0.2)], profile="ft_object", history_length=2, stride=2
        )
        np.testing.assert_array_equal(before, after["features"][0])
        self.assertNotIn("timestamp", query["feature_keys"])
        joint = windows(
            [episode("q", 0.2)], profile="ft_object_joints", history_length=2
        )
        tactile = windows(
            [episode("q", 0.2)], profile="ft_object_joints_tactile", history_length=2
        )
        self.assertGreater(tactile["features"].shape[1], joint["features"].shape[1])
        self.assertGreater(joint["features"].shape[1], query["features"].shape[1])
        with self.assertRaises(ValueError):
            assess(reference, joint, radius=0.1, torque_tolerance_nm=0.05)


if __name__ == "__main__":
    unittest.main()
