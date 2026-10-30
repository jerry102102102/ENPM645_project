"""T0-T8 fixtures and scripted inputs for the existing three-layer pipeline.

Run with python -m phase1b.benchmark. This runner does not implement control
or certify incremental assistance tracking.
"""

import argparse
from dataclasses import asdict, dataclass
import hashlib
import importlib
import inspect
import json
from pathlib import Path

import mujoco
import numpy as np

from phase1b.geometry import rotate_y
from phase1b.hardware import sha
from phase1b.interfaces import AssistanceDecision
from phase1b.pipeline import read_episode, run_episode
from phase1b.simulation import PandaWorld


@dataclass(frozen=True)
class InputSpec:
    amplitude_nm: float = 0.01
    onset_seconds: float = 1.0
    ramp_seconds: float = 0.5
    hold_seconds: float = 1.0
    zero_seconds: float = 1.0
    max_assist_torque_nm: float = 0.02
    max_slew_nm_s: float = 0.05
    human_angle_deg: float = 10.0
    correction_angle_deg: float = 2.0
    disturbance_force_n: float = 2.0

    def __post_init__(self):
        for key, value in asdict(self).items():
            if isinstance(value, bool) or not np.isfinite(value) or value <= 0:
                raise ValueError(f"{key} must be positive and finite")
            if key.endswith("seconds") and not np.isclose(value * 100, round(value * 100)):
                raise ValueError(f"{key} must be a multiple of 0.01 s")
        if self.amplitude_nm > self.max_assist_torque_nm:
            raise ValueError("amplitude exceeds the configured torque bound")
        if 1.5 * self.amplitude_nm / self.ramp_seconds > self.max_slew_nm_s + 1e-12:
            raise ValueError("cubic ramp exceeds the configured slew bound")

    @property
    def pulse_seconds(self):
        return 2 * self.ramp_seconds + self.hold_seconds

    def duration(self, case):
        if case not in tuple(f"T{i}" for i in range(9)):
            raise ValueError("implemented cases are T0-T8; T9 is not defined")
        pulses = 2 if case == "T3" else 1
        return self.onset_seconds + pulses * (self.pulse_seconds + self.zero_seconds)

    def torque(self, case, t):
        self.duration(case)
        if not np.isfinite(t) or t < 0:
            raise ValueError("timestamp must be finite and nonnegative")

        def pulse(start):
            u = np.clip((t - start) / self.ramp_seconds, 0, 1)
            v = np.clip((t - start - self.ramp_seconds - self.hold_seconds)
                        / self.ramp_seconds, 0, 1)
            return self.amplitude_nm * ((3 * u**2 - 2 * u**3) - (3 * v**2 - 2 * v**3))

        if case not in ("T1", "T2", "T3"):
            return 0.0
        first = pulse(self.onset_seconds)
        if case == "T1":
            return float(first)
        if case == "T2":
            return float(-first)
        return float(first - pulse(self.onset_seconds + self.pulse_seconds + self.zero_seconds))


class TorqueInput:
    """Test request source. It reads only the current history timestamp."""

    def __init__(self, case="T0", spec=None):
        self.spec = InputSpec() if spec is None else spec
        self.spec.duration(case)
        self.case = case

    def predict(self, history):
        t = float(history.timestamps[-1])
        return AssistanceDecision(t, "HOLD", self.spec.torque(self.case, t),
                                  None, "scripted_test", True)


class BenchmarkWorld(PandaWorld):
    """Shared S0 and runner-owned human/environment inputs; impedance stays active."""

    state_kind = mujoco.mjtState.mjSTATE_INTEGRATION
    # Development fixture gates, not calibrated controller acceptance thresholds.
    settle_limits = dict(linear_speed_m_s=0.001, angular_speed_rad_s=0.01,
                         joint_speed_rad_s=0.01, penetration_m=0.001,
                         consecutive_steps=100, timeout_seconds=10.0)

    def __init__(self):
        self.fixed_target = None
        self.case = "T0"
        self.spec = InputSpec()
        self.reverse_after_pause = False
        super().__init__("B")

    def environment_targets(self, t, scenario):
        if self.fixed_target is None:
            return super().environment_targets(t, "B")
        angle, angular_speed = 0.0, 0.0
        fe = np.zeros(3)
        spec = self.spec

        def ramp(start):
            u = np.clip((t - start) / spec.ramp_seconds, 0, 1)
            return 3*u*u - 2*u*u*u, 6*u*(1-u) / spec.ramp_seconds

        up, dup = ramp(spec.onset_seconds)
        down, ddown = ramp(spec.onset_seconds + spec.ramp_seconds + spec.hold_seconds)
        amplitude = np.deg2rad(spec.human_angle_deg)
        if self.case in ("T4", "T5"):
            direction = 1 if self.case == "T4" else -1
            angle, angular_speed = direction * amplitude * up, direction * amplitude * dup
        elif self.case == "T6":
            correction = np.deg2rad(spec.correction_angle_deg)
            angle, angular_speed = correction * (up - down), correction * (dup - ddown)
        elif self.case == "T7":
            fe[2] = -spec.disturbance_force_n * up
        elif self.case == "T8":
            direction = -1 if self.reverse_after_pause else 1
            angle = .5 * amplitude * (up + direction * down)
            angular_speed = .5 * amplitude * (dup + direction * ddown)
        if self.case in ("T4", "T5", "T6", "T8") and (angle != 0 or angular_speed != 0):
            radius = rotate_y(angle) @ (self.fixed_target - self.pivot)
            target = self.pivot + radius
            velocity = np.cross([0.0, angular_speed, 0.0], radius)
            if np.linalg.norm(velocity) > self.p["human_target_speed_limit"] + 1e-12:
                raise ValueError("human target exceeds the frozen hardware speed limit")
        else:
            target, velocity = self.fixed_target.copy(), np.zeros(3)
        return self.p["initial_angle"] + angle, target, velocity, fe

    def prepare(self):
        self.settle()
        self.fixed_target = self.d.site_xpos[self.hsite].copy()
        self.pivot = self.d.xipos[self.panel].copy()
        self.refresh(0)
        stable = 0
        limits = self.settle_limits
        for step in range(round(limits["timeout_seconds"] / self.m.opt.timestep)):
            self.step()
            o, d = self.snapshot()
            contact = np.r_[o["contact_state_left"], o["contact_state_right"]]
            good = (
                np.linalg.norm(o["object_linear_velocity"]) < limits["linear_speed_m_s"]
                and np.linalg.norm(o["object_angular_velocity"]) < limits["angular_speed_rad_s"]
                and np.max(np.abs(o["arm_joint_velocity"])) < limits["joint_speed_rad_s"]
                and contact.min() >= 1 and d["penetration"] < limits["penetration_m"]
                and not self.d.warning.number.any()
            )
            stable = stable + 1 if good else 0
            if stable >= limits["consecutive_steps"]:
                break
        else:
            raise RuntimeError("fixed-human fixture did not settle within its declared gates")
        self.origin = float(self.d.time)
        self.tick = 0
        self.refresh(0)
        self.settle_steps = step + 1
        return self.capture()

    def capture(self):
        state = np.empty(mujoco.mj_stateSize(self.m, self.state_kind))
        mujoco.mj_getState(self.m, self.d, state, self.state_kind)
        return dict(integration_state=state, fixed_target=self.fixed_target.copy(),
                    human_rotation_pivot=self.pivot.copy(),
                    nominal_arm_position=self.qtarget.copy(), origin=np.array(self.origin),
                    state_kind=np.array(int(self.state_kind)))

    def restore(self, saved):
        if int(saved["state_kind"]) != int(self.state_kind):
            raise ValueError("unsupported checkpoint state mask")
        self.fixed_target = saved["fixed_target"].copy()
        self.pivot = saved["human_rotation_pivot"].copy()
        self.qtarget = saved["nominal_arm_position"].copy()
        self.origin = float(saved["origin"])
        self.tick = 0
        mujoco.mj_resetData(self.m, self.d)
        mujoco.mj_setState(self.m, self.d, saved["integration_state"], self.state_kind)
        self.refresh(0)


def load_controller(path):
    if path is None:
        return None
    module, name = path.split(":")
    return getattr(importlib.import_module(module), name)()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def physical_guard(world):
    """Stop development runs on hard physical failures at 500 Hz."""
    o, d = world.snapshot()
    if not all(np.isfinite(v).all() for v in o.values()):
        raise RuntimeError("non-finite observation")
    if world.d.warning.number.any():
        raise RuntimeError("MuJoCo numerical warning")
    if min(np.r_[o["contact_state_left"], o["contact_state_right"]]) < 1:
        raise RuntimeError("lost pad contact")
    if d["penetration"] >= world.settle_limits["penetration_m"]:
        raise RuntimeError("pad penetration exceeded development limit")
    q = world.d.qpos[world.qa]
    bounds = world.m.jnt_range[world.arm_joints]
    if np.any(q <= bounds[..., 0]) or np.any(q >= bounds[..., 1]):
        raise RuntimeError("arm joint limit reached")
    pads = sum(world.pad_g, [])
    for c in world.d.contact:
        expected = world.pg in (c.geom1, c.geom2) and (c.geom1 in pads or c.geom2 in pads)
        if not expected and c.dist < -1e-5:
            raise RuntimeError("unexpected penetrating contact")


def summarize(path):
    data, _ = read_episode(path)
    o, n = data["observations"], data["next_observations"]
    d, c = data["diagnostics"], data["controller"]
    # Metrics describe total response. They are deliberately not assistance tracking errors.
    rotations = n["object_orientation"]
    q0 = o["object_orientation"][0]
    angle = 2 * np.arccos(np.clip(np.abs(np.sum(rotations * q0, axis=1)), 0, 1))
    return dict(
        max_orientation_change_rad=float(angle.max()),
        max_height_change_m=float(np.abs(n["object_position"][:, 2] - o["object_position"][0, 2]).max()),
        min_contacts_per_pad=d["contact_counts"].min(axis=(0, 1)).tolist(),
        max_penetration_m=float(d["penetration_m"].max()),
        max_unexpected_penetration_m=float(d["unexpected_penetration_max_m"].max()),
        min_joint_margin_rad=float(d["arm_joint_limit_margin_rad"].min()),
        servo_saturation_fraction=float(c["arm_force_saturated"].mean()),
        warning_count=int(d["warnings"].max(axis=(0, 1)).sum()),
        max_human_force_n=float(np.linalg.norm(data["privileged"]["human_applied_force"], axis=1).max()),
        max_abs_requested_assist_nm=float(np.abs(data["decisions"]["assist_torque_y_nm"]).max()),
        any_assistance_execution_reported=bool(c["assist_reference_executed"].any()),
        assistance_tracking_validated=False,
        expert_labels_validated=False,
    )


def run_benchmark(output, spec=None, cases=("T0", "T1", "T2", "T3"),
                  object_controller=None, dual_arm_controller=None, policy=None,
                  reverse_after_pause=False):
    """Class import paths create fresh controller/filter state for every run."""
    spec = InputSpec() if spec is None else spec
    cases = tuple(cases)
    if not cases or len(cases) != len(set(cases)):
        raise ValueError("provide unique nonempty cases")
    for case in cases:
        spec.duration(case)
    if policy is not None and not any(case in ("T4", "T5", "T6", "T7", "T8") for case in cases):
        raise ValueError("custom policies apply only to T4-T8; T0-T3 use scripted requests")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    report = dict(fixture_version="controller-scenes/1.0", spec=asdict(spec),
                  cases=list(cases), mode="development", deterministic_fixture=True,
                  policy=policy, reverse_after_pause=reverse_after_pause,
                  controller_acceptance="pending_calibrated_thresholds_and_tracking_evidence",
                  object_controller=object_controller, dual_arm_controller=dual_arm_controller,
                  fixture_settle_limits=BenchmarkWorld.settle_limits,
                  runs={}, status="preparing")
    write_json(output / "report.json", report)
    world = None
    try:
        world = BenchmarkWorld()
        world.spec = spec
        world.reverse_after_pause = reverse_after_pause
        saved = world.prepare()
        np.savez_compressed(output / "initial_state.npz", **saved)
        report["initial_state_sha256"] = sha(output / "initial_state.npz")
        report["post_latch_settle_steps"] = world.settle_steps
        report["fixed_human_target_world_m"] = saved["fixed_target"].tolist()
        report["input_spec_sha256"] = hashlib.sha256(
            json.dumps(asdict(spec), sort_keys=True).encode()).hexdigest()
        # T0 provides the 4 s baseline. T3 needs a separate 7 s zero baseline.
        plans = {case: (case, spec.duration(case)) for case in cases}
        baseline_names = {}
        for case in cases:
            if case in ("T1", "T2", "T3"):
                name = "T0" if case in ("T1", "T2") else "T0_for_T3"
                plans.setdefault(name, ("T0", spec.duration(case)))
                baseline_names[case] = name
            elif policy is not None and case != "T0":
                name = case + "_zero"
                plans[name] = (case, spec.duration(case))
                baseline_names[case] = name
        initial_observation = None
        for name, (case, duration) in plans.items():
            report["active_run"] = name
            write_json(output / "report.json", report)
            world.case = case
            world.restore(saved)
            observation, _ = world.snapshot()
            if initial_observation is None:
                initial_observation = observation
            for key in observation:
                np.testing.assert_array_equal(observation[key], initial_observation[key])
            cooperation = case in ("T4", "T5", "T6", "T7", "T8")
            components = {"policy": load_controller(policy) if cooperation and policy and name == case else TorqueInput(case, spec),
                          "object_controller": load_controller(object_controller),
                          "dual_arm_controller": load_controller(dual_arm_controller)}
            hashes = {}
            for role, component in components.items():
                if component is not None:
                    source = Path(inspect.getfile(type(component)))
                    hashes[role] = dict(path=str(source), sha256=sha(source))
            verification = run_episode(
                output / name, "B", duration, world=world,
                step_guard=physical_guard,
                episode_context=dict(
                    task_id=case, run_id=name, fixture_version=report["fixture_version"],
                    input_spec=asdict(spec), reverse_after_pause=reverse_after_pause,
                    initial_state_sha256=report["initial_state_sha256"],
                    fixed_human_target_world_m=report["fixed_human_target_world_m"],
                    controller_source_hashes=hashes,
                ),
                max_assist_torque_nm=spec.max_assist_torque_nm, **components)
            report["runs"][name] = dict(
                verification=verification, metrics=summarize(output / name),
                controller_source_hashes=hashes, initial_observation_equal=True,
                baseline=baseline_names.get(name),
                verdict="tracking_unverified" if not cooperation else "cooperation_unverified")
            write_json(output / "report.json", report)
        for case in cases:
            if case not in baseline_names:
                continue
            active, _ = read_episode(output / case)
            baseline, _ = read_episode(output / baseline_names[case])
            np.savez_compressed(
                output / f"{case}_comparison.npz",
                timestamp=active["next_observations"]["timestamp"],
                requested_assist_nm=active["decisions"]["assist_torque_y_nm"],
                total_contact_ty_difference_nm=(active["next_observations"]["object_contact_wrench"][:, 4]
                                               - baseline["next_observations"]["object_contact_wrench"][:, 4]),
                position_divergence_m=(active["next_observations"]["object_position"]
                                       - baseline["next_observations"]["object_position"]),
                orientation_quaternion_active=active["next_observations"]["object_orientation"],
                orientation_quaternion_baseline=baseline["next_observations"]["object_orientation"],
                tracking_valid=np.zeros(len(active["next_observations"]["timestamp"]), bool),
            )
        report["status"] = "completed"
    except Exception as error:
        report["status"] = "failed"
        report["error"] = f"{type(error).__name__}: {error}"
        if world is not None:
            np.savez_compressed(output / "failure_state.npz", qpos=world.d.qpos,
                                qvel=world.d.qvel, ctrl=world.d.ctrl, time=world.d.time)
        write_json(output / "report.json", report)
        raise
    write_json(output / "report.json", report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cases", nargs="+", choices=tuple(f"T{i}" for i in range(9)),
                        default=["T0", "T1", "T2", "T3"])
    for key, value in asdict(InputSpec()).items():
        parser.add_argument("--" + key.replace("_", "-"), type=float, default=value)
    parser.add_argument("--policy", help="module:Class for T4-T8; default is zero assistance")
    parser.add_argument("--reverse-after-pause", action="store_true", help="T8 reverses to its starting target instead of resuming")
    parser.add_argument("--object-controller", help="module:Class, fresh no-argument instance per run")
    parser.add_argument("--dual-arm-controller", help="module:Class, fresh no-argument instance per run")
    args = parser.parse_args()
    spec = InputSpec(**{key: getattr(args, key) for key in asdict(InputSpec())})
    report = run_benchmark(args.output, spec, args.cases, args.object_controller, args.dual_arm_controller, args.policy, args.reverse_after_pause)
    print(json.dumps({"output": str(args.output), "status": report["status"],
                      "controller_acceptance": report["controller_acceptance"]}, indent=2))


if __name__ == "__main__":
    main()
