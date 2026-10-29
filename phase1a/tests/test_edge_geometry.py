"""Edge semantics checked against compiled geometry and actual force application."""

import json
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from phase0_tactile.scripts.run_experiment import rotate_y
from phase0_v2.simulation.dual_gripper import MODEL, DualPilot
from phase1a.scripts.audit_grasp_geometry import snapshot
from phase1a.simulation.edge_gripper import EdgeGraspPilot
from phase1a.simulation.scenario_loader import (
    CONFIG,
    EDGE_CONFIG,
    REPO,
    hardware_binding,
    load_scenario,
    make_sim,
)


class EdgeGeometry(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_scenario(EDGE_CONFIG)

    def test_only_intended_mjcf_attributes_change(self):
        old = ET.parse(MODEL).getroot()
        new = ET.parse(REPO / self.config["model_path"]).getroot()
        new.set("model", old.get("model"))
        for side in ("left", "right"):
            path = f"./worldbody/body[@name='{side}']"
            new.find(path).set("pos", old.find(path).get("pos"))
        path = "./worldbody/body[@name='panel']/site[@name='human_grasp']"
        new.find(path).set("pos", old.find(path).get("pos"))
        self.assertEqual(ET.tostring(old), ET.tostring(new))

    def test_nominal_edge_anchors_and_physical_pad_regions(self):
        sim = make_sim(self.config)
        state = snapshot(sim)
        for side, x in [("left", -0.2), ("right", 0.2)]:
            np.testing.assert_allclose(
                state["grasp_centers_panel_local_m"][side], [x, 0.1, 0], atol=1e-12
            )
            for pad, z in [("upper", 0.02), ("lower", -0.02)]:
                geom = state["pads"][f"{side}_{pad}_pad"]
                np.testing.assert_allclose(
                    geom["center_panel_local_m"], [x, 0.1, z], atol=1e-12
                )
                lo, hi = np.array(geom["bounds_panel_local_m"])
                np.testing.assert_allclose([lo[1], hi[1]], [0.03, 0.17], atol=1e-12)
                self.assertAlmostEqual(
                    lo[2] if pad == "upper" else hi[2],
                    0.01 if pad == "upper" else -0.01,
                )
        np.testing.assert_allclose(
            state["human_point_panel_local_m"], [0.27, -0.1, 0], atol=1e-12
        )
        self.assertEqual(sim.m.neq, 0)
        self.assertTrue(
            all(
                sim.m.jnt_type[sim.m.body_jntadr[b]] == mujoco.mjtJoint.mjJNT_FREE
                for b in [sim.panel, *sim.roots]
            )
        )

    def test_human_target_uses_new_site_without_preload_geometry_error(self):
        sim = make_sim(self.config)
        truth, _ = sim.forces(-1.0, "B")
        expected = sim.center + rotate_y(
            self.config["parameters"]["initial_angle"]
        ) @ np.array([0.27, -0.1, 0])
        np.testing.assert_allclose(
            truth["human_target_trajectory"], expected, atol=1e-14
        )
        np.testing.assert_allclose(
            truth["human_target_trajectory"], sim.d.site_xpos[sim.hsite], atol=1e-14
        )
        np.testing.assert_allclose(truth["human_applied_wrench"], 0, atol=1e-12)

    def test_old_geometry_with_new_adapter_matches_original_dynamics(self):
        p = load_scenario(CONFIG)["parameters"]
        a = DualPilot(p)
        b = EdgeGraspPilot(p, MODEL)
        for k in range(150):
            a.forces(k * 0.002, "A")
            b.forces(k * 0.002, "A")
            mujoco.mj_step(a.m, a.d)
            mujoco.mj_step(b.m, b.d)
        np.testing.assert_allclose(a.d.qpos, b.d.qpos, atol=1e-10, rtol=0)
        np.testing.assert_allclose(a.d.qvel, b.d.qvel, atol=1e-10, rtol=0)

    def test_settled_contacts_on_panel_surfaces(self):
        sim = make_sim(self.config)
        sim.settle()
        sim.forces(0.0, "B")
        state = snapshot(sim)
        self.assertGreater(len(state["active_contacts"]), 0)
        for contact in state["active_contacts"]:
            p = np.array(contact["position_panel_local_m"])
            self.assertLessEqual(abs(p[0]), 0.3 + 0.001)
            self.assertLessEqual(abs(p[1]), 0.1 + 0.001)
            self.assertLess(abs(abs(p[2]) - 0.01), 0.001)
            self.assertGreater(p[1], 0.0)
        np.testing.assert_allclose(
            state["human_point_panel_local_m"], [0.27, -0.1, 0], atol=1e-12
        )

    def test_frozen_hardware_rejects_mass_or_contact_parameter_changes(self):

        sim = make_sim(self.config)
        binding = hardware_binding(self.config, sim)
        self.assertEqual(binding["hardware_version"], "1.0.0")
        sim.m.body_mass[sim.roots[0]] += 0.01
        with self.assertRaises(ValueError):
            hardware_binding(self.config, sim)
        sim = make_sim(self.config)
        sim.m.geom_friction[sim.pad_g[0][0], 0] += 0.01
        with self.assertRaises(ValueError):
            hardware_binding(self.config, sim)

        sim = make_sim(self.config)
        sim.p["kh"] += 1.0
        with self.assertRaises(ValueError):
            hardware_binding(self.config, sim)

    def test_frozen_hardware_file_cannot_be_silently_changed(self):
        from phase1a.simulation.hardware_contract import validate_hardware
        from phase1a.simulation.scenario_loader import digest

        path = REPO / "phase1a/config/hardware_v1.json"
        with tempfile.TemporaryDirectory() as temp:
            altered = Path(temp) / "hardware.json"
            profile = json.loads(path.read_text())
            profile["hardware"]["human"]["force_norm_limit_N"] = 100.0
            altered.write_text(json.dumps(profile))
            with self.assertRaises(ValueError):
                validate_hardware(
                    make_sim(self.config),
                    self.config,
                    REPO / self.config["model_path"],
                    altered,
                    digest(path),
                )


if __name__ == "__main__":
    unittest.main()
