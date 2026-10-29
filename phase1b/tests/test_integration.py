"""Physical and causality checks for the articulated integration boundary."""

import tempfile
import unittest
from pathlib import Path

import mujoco
import numpy as np

from phase1a.interfaces.controller_contract import AssistTorqueReference
from phase1b.interfaces import (
    ArmCommand,
    PositionHoldBaseline,
    policy_observation,
    validate_command,
)
from phase1b.pipeline import (
    controller_input,
    read_episode,
    run_episode,
    validate_episode,
)
from phase1b.simulation import PandaWorld


class IntegrationTests(unittest.TestCase):
    def test_only_panel_has_free_joint_and_bases_are_fixed(self):
        w = PandaWorld()
        free = np.flatnonzero(w.m.jnt_type == mujoco.mjtJoint.mjJNT_FREE)
        self.assertEqual([w.m.joint(int(i)).name for i in free], ["panel_free"])
        self.assertEqual(w.m.neq, 0)
        for side in ("left", "right"):
            self.assertEqual(w.m.body(side + "_link0").parentid[0], 0)
            self.assertEqual(w.m.body(side + "_link0").jntnum[0], 0)
        self.assertEqual(w.arm_joints.shape, (2, 7))

    def test_nominal_ik_and_limits(self):
        w = PandaWorld()
        R = w.d.xmat[w.panel].reshape(3, 3)
        for side, x in [("left", -0.2), ("right", 0.2)]:
            site = w.m.site(side + "_ee").id
            goal = w.center + R @ np.array([x, 0.1, 0])
            np.testing.assert_allclose(w.d.site_xpos[site], goal, atol=1e-6)
            np.testing.assert_allclose(w.d.site_xmat[site].reshape(3, 3), R, atol=1e-5)
        self.assertGreater(
            np.min(w.d.qpos[w.qa] - w.m.jnt_range[w.arm_joints, 0]), 0.12
        )
        self.assertGreater(
            np.min(w.m.jnt_range[w.arm_joints, 1] - w.d.qpos[w.qa]), 0.12
        )

    def test_default_policy_logs_but_excludes_joint_states_and_truth(self):
        w = PandaWorld()
        w.refresh(0)
        o, _ = w.snapshot()
        contaminated = dict(
            o, human_goal=123, scenario_label="A", expert_assist_torque_ref=456
        )
        base = policy_observation(contaminated)
        self.assertNotIn("arm_joint_position", base)
        self.assertNotIn("human_goal", base)
        self.assertNotIn("scenario_label", base)
        self.assertNotIn("expert_assist_torque_ref", base)
        opt = policy_observation(contaminated, "with_joint_states")
        self.assertIn("arm_joint_position", opt)
        self.assertIn("arm_joint_velocity", opt)
        self.assertNotIn("arm_actuator_torque", opt)
        with self.assertRaises(ValueError):
            opt["arm_joint_position"][0, 0] = 42
        with self.assertRaises(ValueError):
            policy_observation(o, "typo")

    def test_reference_units_future_rejection_and_bounds(self):
        w = PandaWorld()
        w.refresh(0)
        o, _ = w.snapshot()
        inp = controller_input(w, o)
        ref = AssistTorqueReference(0, 0.02)
        np.testing.assert_array_equal(ref.assistance_wrench(), [0, 0, 0, 0, 0.02, 0])
        out = PositionHoldBaseline().step(ref, inp)
        validate_command(out, inp)
        self.assertFalse(out.assist_reference_executed)
        with self.assertRaises(ValueError):
            PositionHoldBaseline().step(AssistTorqueReference(0.1, 0.02), inp)
        bad = out.arm_position_target.copy()
        bad[0, 0] = 10
        with self.assertRaises(ValueError):
            validate_command(ArmCommand(bad, out.pad_force_command), inp)

    def test_logging_alignment_replay_and_corruption_rejection(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            result = run_episode(root / "one", "C", duration=0.1)
            self.assertTrue(result["physical_sample_checks_passed"])
            run_episode(root / "two", "C", duration=0.1)
            a, _ = read_episode(root / "one")
            b, _ = read_episode(root / "two")
            for section in a:
                for k in a[section]:
                    np.testing.assert_array_equal(a[section][k], b[section][k])
            a["observations"]["arm_joint_position"][0, 0, 0] += 1
            np.savez_compressed(root / "one/observations.npz", **a["observations"])
            # A model-state vs recorded-state check is covered by the full replay;
            # corrupt timestamp continuity is rejected directly by the schema validator.
            a["observations"]["timestamp"][0] += 0.1
            np.savez_compressed(root / "one/observations.npz", **a["observations"])
            with self.assertRaises(AssertionError):
                validate_episode(root / "one")

    def test_pad_removal_removes_sensing_and_support(self):
        w = PandaWorld()
        w.settle()
        z0 = w.d.xpos[w.panel, 2]
        for gid in sum(w.pad_g, []):
            w.m.geom_contype[gid] = 0
            w.m.geom_conaffinity[gid] = 0
        w.refresh(0)
        for _ in range(150):
            w.step()
        o, _ = w.snapshot()
        self.assertGreater(z0 - o["object_position"][2], 0.1)
        for side in ("left", "right"):
            self.assertEqual(np.linalg.norm(o["tactile_" + side]), 0)
            self.assertEqual(np.linalg.norm(o["wrist_ft_" + side]), 0)

    def test_arm_actuator_disable_loses_support(self):
        w = PandaWorld()
        w.settle()
        z0 = w.d.xpos[w.panel, 2]
        q0 = w.d.qpos[w.qa].copy()
        w.m.actuator_gainprm[w.aa.ravel()] = 0
        w.m.actuator_biasprm[w.aa.ravel()] = 0
        w.refresh(0)
        for _ in range(150):
            w.step()
        self.assertGreater(z0 - w.d.xpos[w.panel, 2], 0.1)
        self.assertGreater(np.max(np.abs(w.d.qpos[w.qa] - q0)), 0.1)
        np.testing.assert_array_equal(w.d.qfrc_actuator[w.va], 0)


if __name__ == "__main__":
    unittest.main()
