"""Manual/scripted Virtual Human targets; uses the existing force model and pipeline."""

import argparse
from contextlib import nullcontext
import copy
import json
from pathlib import Path
from queue import SimpleQueue
import time

import mujoco
import numpy as np

from phase1b.benchmark import BenchmarkWorld, load_controller, physical_guard, summarize, write_json
from phase1b.geometry import rotate_y
from phase1b.hardware import sha
from phase1b.interfaces import AssistanceDecision
from phase1b.pipeline import read_episode, run_episode

VERSION = "virtual-human-input/1.0"


class PassivePolicy:
    """No intent inference or active assistance; never consume operator annotations."""

    def predict(self, history):
        return AssistanceDecision(float(history.timestamps[-1]), "UNCERTAIN", 0.,
                                  None, "scripted_test", True)


class HandTargets:
    """Causal, continuous position targets. Events describe reported operator intent.

    Angles are offsets about world Y from the settled hand target, not panel commands.
    HOLD cancels the remaining goal and fixes the current target (not measured pose).
    """

    def __init__(self, speed_deg_s=5.0, limit_deg=10.0):
        if any(isinstance(x, bool) or not np.isfinite(x) or x <= 0
               for x in (speed_deg_s, limit_deg)):
            raise ValueError("positive finite speed and angle bound required")
        self.speed = float(speed_deg_s)
        self.limit = float(limit_deg)
        self.events = []
        self._starts = []
        self.add(0.0, "HOLD")

    def sample(self, t):
        if not np.isfinite(t) or t < 0:
            raise ValueError("nonnegative finite simulation time required")
        # ponytail: short pilot event lists; index/bisect only if long sessions need it.
        index = max(i for i, event in enumerate(self.events) if event["timestamp"] <= t + 1e-12)
        event, start = self.events[index], self._starts[index]
        delta = event["goal_offset_deg"] - start
        travel = self.speed * max(0, t - event["timestamp"])
        angle = start + np.sign(delta) * min(abs(delta), travel)
        velocity = np.sign(delta) * self.speed if travel < abs(delta) - 1e-12 else 0.0
        return float(angle), float(velocity), event

    def add(self, timestamp, mode, goal_offset_deg=None, **provenance):
        if (isinstance(timestamp, bool) or not np.isfinite(timestamp) or timestamp < 0
                or not np.isclose(timestamp / .002, round(timestamp / .002), atol=1e-9, rtol=0)):
            raise ValueError("event timestamp must be on a nonnegative 500 Hz tick")
        if self.events and timestamp < self.events[-1]["timestamp"]:
            raise ValueError("events must be chronological")
        if mode not in ("ROTATE_POS", "ROTATE_NEG", "HOLD"):
            raise ValueError("unknown human input mode")
        start = self.sample(timestamp)[0] if self.events else 0.0
        if mode == "HOLD":
            if goal_offset_deg is not None and not np.isclose(goal_offset_deg, start, atol=1e-12, rtol=0):
                raise ValueError("HOLD must latch the current target")
            goal_offset_deg = start
        if (goal_offset_deg is None or isinstance(goal_offset_deg, bool)
                or not np.isfinite(goal_offset_deg) or abs(goal_offset_deg) > self.limit):
            raise ValueError("goal must be finite and within the declared angular bound")
        if ((mode == "ROTATE_POS" and goal_offset_deg <= start)
                or (mode == "ROTATE_NEG" and goal_offset_deg >= start)):
            raise ValueError("rotation goal must match requested direction")
        if set(provenance) - {"key", "received_wall_seconds", "applied_wall_seconds"}:
            raise ValueError("unknown event provenance")
        self.events.append(dict(timestamp=float(timestamp), mode=mode,
                                goal_offset_deg=float(goal_offset_deg), **provenance))
        self._starts.append(start)


class HumanWorld(BenchmarkWorld):
    def __init__(self):
        self.hand = None
        self.poll = None
        self.last_input_time = -1.0
        super().__init__()

    def environment_targets(self, t, scenario):
        if self.hand is None:
            return super().environment_targets(t, scenario)
        # refresh() can run repeatedly at the same tick. Never resample live input then.
        if t > self.last_input_time:
            if self.poll is not None:
                self.poll(t)
            self.last_input_time = t
        angle, speed, _ = self.hand.sample(t)
        radius = rotate_y(np.deg2rad(angle)) @ (self.fixed_target - self.pivot)
        velocity = np.cross([0, np.deg2rad(speed), 0], radius)
        if np.linalg.norm(velocity) > self.p["human_target_speed_limit"] + 1e-12:
            raise ValueError("human target exceeds frozen hardware speed limit")
        return self.initial_pitch + np.deg2rad(angle), self.pivot + radius, velocity, np.zeros(3)

    def refresh(self, t, scenario=None):
        truth = super().refresh(t, scenario)
        if self.hand is not None:
            angle, _, event = self.hand.sample(t)
            _, _, velocity, _ = self.environment_targets(t, self.scenario)
            truth.update(
                ground_truth_intent=event["mode"],
                human_goal_angle=self.initial_pitch + np.deg2rad(event["goal_offset_deg"]),
                human_target_angle=self.initial_pitch + np.deg2rad(angle),
                human_target_velocity=velocity,
                human_event_timestamp=event["timestamp"],
                human_input_source=self.input_source,
            )
        return truth

    def attach(self, hand, source):
        self.hand, self.input_source = hand, source
        R = self.d.xmat[self.panel].reshape(3, 3)
        self.initial_pitch = float(np.arctan2(-R[2, 0], R[0, 0]))
        self.refresh(0.0)


def run_human(output, *, duration=10.0, mode="demo", replay=None, show=False,
              object_controller=None, dual_arm_controller=None):
    if mode not in ("demo", "manual", "replay") or (mode == "manual" and not show):
        raise ValueError("manual input requires --viewer; mode must be demo/manual/replay")
    if not np.isfinite(duration) or duration <= 0 or not np.isclose(duration * 100, round(duration * 100)):
        raise ValueError("positive duration on a 100 Hz boundary required")
    source = None
    if mode == "replay":
        if replay is None:
            raise ValueError("replay requires a previous completed session directory")
        replay = Path(replay)
        source = json.loads((replay / "session.json").read_text())
        if source["version"] != VERSION or source["status"] != "completed":
            raise ValueError("replay requires a completed compatible session")
        for name, digest in source["replay_hashes"].items():
            if sha(replay / name) != digest:
                raise ValueError(f"replay hash mismatch: {name}")
        duration = source["duration_seconds"]
    hand = HandTargets() if source is None else HandTargets(**source["target_parameters"])
    events = [] if source is None else json.loads((replay / "events.json").read_text())
    if source is not None:
        if not events or events[0] != hand.events[0]:
            raise ValueError("replay must start with the canonical zero-offset HOLD")
        events = events[1:]
    if mode == "demo":
        # Deliberate mid-motion stop, reverse, then hold. No robot trajectory is prescribed.
        events = [dict(timestamp=1., mode="ROTATE_POS", goal_offset_deg=5.),
                  dict(timestamp=1.6, mode="HOLD"),
                  dict(timestamp=3., mode="ROTATE_NEG", goal_offset_deg=-3.),
                  dict(timestamp=4.2, mode="HOLD")]
    for event in events:
        hand.add(**event)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    session = dict(version=VERSION, status="preparing", mode=mode,
                   replay_of=str(replay) if source else None,
                   target_parameters=dict(speed_deg_s=hand.speed, limit_deg=hand.limit),
                   object_controller=object_controller, dual_arm_controller=dual_arm_controller,
                   tracking_validated=False, expert_labels_validated=False)
    write_json(output / "session.json", session)
    world = None
    try:
        world = HumanWorld()
        saved = world.prepare()
        if source is not None:
            if source["model_hash"] != world.config["model_sha256"] or source["config_hash"] != sha(Path(__file__).parent / "config/dual_panda_v1.json"):
                raise ValueError("replay model/config differs")
            with np.load(replay / "initial_state.npz", allow_pickle=False) as checkpoint:
                saved = dict(checkpoint)
        # Original and replay both enter through the same checkpoint restoration.
        world.restore(saved)
        np.savez_compressed(output / "initial_state.npz", **saved)
        world.attach(hand, "manual_keyboard" if mode == "manual" else
                     source["input_source"] if source else "scripted_input_test")
        session.update(input_source=world.input_source, model_hash=world.config["model_sha256"],
                       config_hash=sha(Path(__file__).parent / "config/dual_panda_v1.json"),
                       hand_target_origin_world_m=world.fixed_target.tolist(),
                       rotation_pivot_world_m=world.pivot.tolist(),
                       initial_panel_pitch_rad=world.initial_pitch,
                       human_stiffness_n_m=world.p["kh"], human_damping_ns_m=world.p["dh"],
                       human_force_limit_n=world.p["human_force_limit"],
                       target_speed_limit_m_s=world.p["human_target_speed_limit"],
                       source_sha256=sha(Path(__file__)))
        pending = SimpleQueue()
        session["raw_inputs"] = []
        stopped = False
        wall_start = time.monotonic()

        def receive(key):
            if mode == "manual" and key in (ord("J"), ord("K"), ord("L"), ord("Q")):
                pending.put((key, time.monotonic() - wall_start))

        def poll(t):
            nonlocal stopped
            while not pending.empty():
                key, received = pending.get()
                raw = dict(key=chr(key), received_wall_seconds=received,
                           applied_wall_seconds=time.monotonic() - wall_start,
                           simulation_timestamp=t, status="applied")
                session["raw_inputs"].append(raw)
                if key == ord("Q"):
                    raw["status"] = "stop_requested"
                    stopped = True
                    continue
                angle = hand.sample(t)[0]
                direction = 1 if key == ord("L") else -1
                goal = float(np.clip(angle + direction * 5., -hand.limit, hand.limit))
                human_mode = "HOLD" if key == ord("K") else "ROTATE_POS" if direction > 0 else "ROTATE_NEG"
                if human_mode != "HOLD" and abs(goal - angle) < 1e-12:
                    raw["status"] = "at_angle_limit"
                    continue
                hand.add(t, human_mode, None if human_mode == "HOLD" else goal,
                         key=chr(key), received_wall_seconds=received,
                         applied_wall_seconds=raw["applied_wall_seconds"])
                print(f"t={t:.3f}s {human_mode}: target goal {hand.events[-1]['goal_offset_deg']:+.2f} deg", flush=True)
        world.poll = poll if mode == "manual" else None
        viewer_context = nullcontext(None)
        if show:
            import mujoco.viewer
            # Isolated display state: viewer mouse drags cannot perturb the experiment.
            display_model = copy.copy(world.m)
            display_data = mujoco.MjData(display_model)
            viewer_context = mujoco.viewer.launch_passive(display_model, display_data, key_callback=receive)
        with viewer_context as viewer:
            if viewer is not None:
                viewer.cam.lookat[:] = [0, .3, .65]
                viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 1.8, 90, -25
                print("J: negative goal | L: positive goal | K: hold target here | Q/close: save and exit", flush=True)
            pace_start = time.monotonic()

            def guard(w):
                physical_guard(w)
                if viewer is not None and viewer.is_running():
                    if w.tick % 20 == 0:  # 25 Hz display, 500 Hz physics.
                        from phase1b.demos.render_helpers import arrow, marker
                        display_data.qpos[:] = w.d.qpos
                        display_data.qvel[:] = w.d.qvel
                        mujoco.mj_forward(display_model, display_data)
                        with viewer.lock():
                            viewer.user_scn.ngeom = 0
                            marker(viewer.user_scn, w.truth["human_target_trajectory"], (49, 224, 205))
                            point = w.truth["human_application_point"]
                            marker(viewer.user_scn, point, (255, 90, 90))
                            arrow(viewer.user_scn, point, point + .03 * w.truth["human_applied_force"], (255, 90, 90))
                        viewer.sync()
                    time.sleep(max(0, pace_start + w.tick * .002 - time.monotonic()))

            verification = run_episode(output / "episode", "B", duration, world=world,
                policy=PassivePolicy(), object_controller=load_controller(object_controller),
                dual_arm_controller=load_controller(dual_arm_controller), step_guard=guard,
                stop_requested=lambda: stopped or (viewer is not None and not viewer.is_running()),
                episode_context=dict(task_id="HUMAN_INPUT_PILOT", input_contract=VERSION,
                                     input_source=world.input_source, formal_demonstration=False))
        data, meta = read_episode(output / "episode")
        # Explicit teacher stream for shadow analysis only. Never enters policy/controller inputs.
        private = data["privileged"]
        np.savez_compressed(output / "teacher_inputs.npz", **{
            key: private[key] for key in ("timestamp", "ground_truth_intent", "human_goal_angle", "external_disturbance_wrench")})
        session.update(status="completed", duration_seconds=meta["duration_seconds"],
                       wall_duration_seconds=time.monotonic() - wall_start,
                       verification=verification, metrics=summarize(output / "episode"))
    except Exception as error:
        session.update(status="failed", error=f"{type(error).__name__}: {error}")
        if world is not None:
            np.savez_compressed(output / "failure_state.npz", qpos=world.d.qpos, qvel=world.d.qvel, time=world.d.time)
        raise
    finally:
        write_json(output / "events.json", hand.events)
        session["replay_hashes"] = {name: sha(output / name) for name in ("events.json", "initial_state.npz") if (output / name).exists()}
        write_json(output / "session.json", session)
    return session


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--mode", choices=("demo", "manual", "replay"), default="demo")
    parser.add_argument("--replay", type=Path)
    parser.add_argument("--duration", type=float, default=10.)
    parser.add_argument("--viewer", dest="show", action="store_true")
    parser.add_argument("--object-controller", help="module:Class; same public controller interface")
    parser.add_argument("--dual-arm-controller", help="module:Class; same public controller interface")
    args = parser.parse_args()
    print(json.dumps(run_human(**vars(args)), indent=2))


if __name__ == "__main__":
    main()
