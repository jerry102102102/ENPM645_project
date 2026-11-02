"""Manual/scripted Virtual Human targets; uses the existing force model and pipeline."""

import argparse
from contextlib import nullcontext
from bisect import bisect_right
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
from phase1b.pipeline import read_episode, run_episode, stack
from phase1b.simulation import CONFIG, ROOT

VERSION = "virtual-human-input/2.0"


def tactile_image(observation, baseline, truth):
    """Four pad grids from the same physics sample; fixed color scales, N/bin."""
    import matplotlib
    from PIL import Image, ImageDraw
    from phase1b.demos.render_helpers import font

    image = Image.new("RGB", (760, 382), "#111923")
    draw = ImageDraw.Draw(image)
    force = truth["human_applied_force"]
    draw.text((12, 6), f"t={observation['timestamp']:.2f}s  {truth['ground_truth_intent']}  "
              f"Human Fxyz=[{force[0]:+.2f}, {force[1]:+.2f}, {force[2]:+.2f}] N",
              font=font(16), fill="white")
    omega = np.rad2deg(truth['human_target_angular_velocity_y'])
    alpha = np.rad2deg(truth['human_target_angular_acceleration_y'])
    measured = np.rad2deg(observation['object_angular_velocity'][1])
    phase = 'HOLD' if abs(omega)+abs(alpha) < 1e-8 else 'CRUISE' if abs(alpha) < 1e-8 else 'BRAKE' if omega*alpha < 0 else 'ACCEL'
    timing = truth.get('follower_timing')
    motion_line = f"{phase} | speed target/actual: {omega:+.1f} / {measured:+.1f} deg/s | accel: {alpha:+.0f} deg/s^2"
    if timing is not None:
        age = ((observation['timestamp']-timing['source_sample_timestamp'])*1000
               if timing['reference_available'] else np.nan)
        motion_line = (f"Hand/panel: {omega:+.1f}/{measured:+.1f} deg/s | "
                       f"follow ref: {timing['follower_target_angle_deg']:+.1f} deg | age: {age:.0f} ms")
    draw.text((12, 29), motion_line,
              font=font(14), fill="#63ded0")
    draw.text((12, 51), "J/L: -/+5 deg   U/O: -/+1 deg   K: brake & hold   Q: save & exit",
              font=font(14), fill="white")
    small, label = font(11), font(14)
    for n, (side, pad) in enumerate((("left", 0), ("left", 1), ("right", 0), ("right", 1))):
        x = 12 + n * 187
        values = observation["tactile_" + side][pad, 0]
        delta = values - baseline["tactile_" + side][pad, 0]
        draw.text((x, 76), f"{side.title()} {('upper', 'lower')[pad]}", font=label, fill="white")
        for y0, grid, cmap, bounds in ((98, values, "magma", (0, 40)),
                                       (241, delta, "coolwarm", (-5, 5))):
            colors = matplotlib.colormaps[cmap](np.clip((grid - bounds[0]) / (bounds[1] - bounds[0]), 0, 1))
            for row in range(4):
                for col in range(4):
                    px, py = x + 32 * col, y0 + 28 * (3 - row)
                    rgb = tuple(int(v * 255) for v in colors[row, col, :3])
                    draw.rectangle((px, py, px + 31, py + 27), fill=rgb)
                    fg = "black" if sum(rgb) > 420 else "white"
                    draw.text((px + 16, py + 14), f"{grid[row,col]:.1f}", anchor="mm", font=small, fill=fg)
        draw.text((x, 212), f"Normal: {values.sum():.2f} N", font=label, fill="white")
        draw.text((x, 356), f"Change: {delta.sum():+.2f} N", font=label, fill="white")
    draw.text((145, 116), "0-40\nN/bin", font=small, fill="white")
    draw.text((145, 257), "+/-5\nN/bin", font=small, fill="white")
    return np.asarray(image)


class PassivePolicy:
    """No intent inference or active assistance; never consume operator annotations."""

    def predict(self, history):
        return AssistanceDecision(float(history.timestamps[-1]), "UNCERTAIN", 0.,
                                  None, "scripted_test", True)


class HandTargets:
    """Bounded-acceleration target motion, replanned from current position/velocity.

    HOLD brakes to a reachable stop; requested stop position is recorded separately.
    Acceleration can jump at phase boundaries (jerk is not bounded).
    """

    def __init__(self, speed_deg_s=15., limit_deg=10., accel_deg_s2=60.):
        if any(isinstance(x, bool) or not np.isfinite(x) or x <= 0
               for x in (speed_deg_s, limit_deg, accel_deg_s2)):
            raise ValueError("positive finite speed, acceleration and angle bound required")
        self.speed, self.limit, self.accel = map(float, (speed_deg_s, limit_deg, accel_deg_s2))
        self.events, self._times, self._plans = [], [], []
        self.add(0., "HOLD")

    def state(self, t):
        if not np.isfinite(t) or t < 0:
            raise ValueError("nonnegative finite simulation time required")
        index = bisect_right(self._times, t + 1e-12) - 1
        event = self.events[index]
        elapsed = max(0., t - event['timestamp'])
        for start, duration, x, v, acc in self._plans[index]:
            if elapsed < start + duration - 1e-12:
                dt = max(0., elapsed - start)
                return x + v*dt + .5*acc*dt*dt, v + acc*dt, acc, event
        return event['goal_offset_deg'], 0., 0., event

    def sample(self, t):
        x, v, _, event = self.state(t)
        return x, v, event

    def add(self, timestamp, mode, goal_offset_deg=None, **provenance):
        if (isinstance(timestamp, bool) or not np.isfinite(timestamp) or timestamp < 0
                or not np.isclose(timestamp / .002, round(timestamp / .002), atol=1e-9, rtol=0)):
            raise ValueError("event timestamp must be on a nonnegative 500 Hz tick")
        if self.events and timestamp < self._times[-1]:
            raise ValueError("events must be chronological")
        if mode not in ("ROTATE_POS", "ROTATE_NEG", "HOLD"):
            raise ValueError("unknown human input mode")
        x, v = self.sample(timestamp)[:2] if self.events else (0., 0.)
        requested_stop = x
        stopping_point = x + v*abs(v)/(2*self.accel)
        if mode == 'HOLD':
            if goal_offset_deg is not None and not np.isclose(goal_offset_deg, stopping_point, atol=1e-9, rtol=0):
                raise ValueError("HOLD goal must equal the bounded-acceleration stopping point")
            goal_offset_deg = stopping_point
        if (goal_offset_deg is None or isinstance(goal_offset_deg, bool)
                or not np.isfinite(goal_offset_deg) or abs(goal_offset_deg) > self.limit + 1e-9):
            raise ValueError("goal must be finite and within the declared angular bound")
        goal_offset_deg = float(np.clip(goal_offset_deg, -self.limit, self.limit))
        if ((mode == 'ROTATE_POS' and goal_offset_deg <= x)
                or (mode == 'ROTATE_NEG' and goal_offset_deg >= x)):
            raise ValueError("rotation goal must match requested direction")
        if set(provenance) - {'key', 'received_wall_seconds', 'applied_wall_seconds', 'requested_stop_offset_deg'}:
            raise ValueError("unknown event provenance")
        if 'requested_stop_offset_deg' in provenance and (
                mode != 'HOLD' or not np.isclose(provenance['requested_stop_offset_deg'], requested_stop, atol=1e-9, rtol=0)):
            raise ValueError("invalid recorded stop request")
        if mode == 'HOLD':
            provenance['requested_stop_offset_deg'] = float(requested_stop)
        plan, elapsed = [], 0.

        def segment(duration, acceleration):
            nonlocal x, v, elapsed
            if duration > 1e-12:
                plan.append((elapsed, duration, x, v, acceleration))
                x += v*duration + .5*acceleration*duration**2
                v += acceleration*duration
                elapsed += duration

        # Brake before reversing, or if the new goal is inside the stopping distance.
        delta = goal_offset_deg - x
        if abs(v) > 1e-12 and (mode == 'HOLD' or delta*v <= 0 or abs(delta) < v*v/(2*self.accel)):
            segment(abs(v)/self.accel, -float(np.sign(v))*self.accel)
            v = 0.
        delta = goal_offset_deg - x
        if abs(delta) > 1e-10:
            direction = float(np.sign(delta))
            initial_speed = direction*v
            peak = min(self.speed, np.sqrt(self.accel*abs(delta) + .5*initial_speed**2))
            accelerate = max(0., (peak - initial_speed)/self.accel)
            distance_acc = (peak**2 - initial_speed**2)/(2*self.accel)
            distance_dec = peak**2/(2*self.accel)
            cruise = max(0., (abs(delta) - distance_acc - distance_dec)/peak)
            segment(accelerate, direction*self.accel)
            segment(cruise, 0.)
            segment(peak/self.accel, -direction*self.accel)
        self.events.append(dict(timestamp=float(timestamp), mode=mode,
                                goal_offset_deg=goal_offset_deg, **provenance))
        self._times.append(float(timestamp))
        self._plans.append(plan)


class HumanWorld(BenchmarkWorld):
    def __init__(self, config_path=None):
        self.hand = None
        self.poll = None
        self.last_input_time = -1.0
        super().__init__(config_path=config_path)

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
            angle, omega, alpha, event = self.hand.state(t)
            _, _, velocity, _ = self.environment_targets(t, self.scenario)
            truth.update(
                ground_truth_intent=event["mode"],
                human_goal_angle=self.initial_pitch + np.deg2rad(event["goal_offset_deg"]),
                human_target_angle=self.initial_pitch + np.deg2rad(angle),
                human_target_velocity=velocity,
                human_target_angular_velocity_y=np.deg2rad(omega),
                human_target_angular_acceleration_y=np.deg2rad(alpha),
                human_target_acceleration=(
                    np.cross([0., np.deg2rad(alpha), 0.], truth['human_target_trajectory'] - self.pivot)
                    + np.cross([0., np.deg2rad(omega), 0.], velocity)),
                human_stop_requested_angle=(self.initial_pitch + np.deg2rad(event['requested_stop_offset_deg'])
                                            if event['mode'] == 'HOLD' else np.nan),
                human_stop_requested_valid=event['mode'] == 'HOLD',
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
              object_controller=None, dual_arm_controller=None, follow_input=None,
              speed_deg_s=None, accel_deg_s2=None, limit_deg=None, scene_config=None,
              follow_delay_ms=None):
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
            raise ValueError("replay requires a completed v2 session; v1 used discontinuous velocities")
        for name, digest in source["replay_hashes"].items():
            if sha(replay / name) != digest:
                raise ValueError(f"replay hash mismatch: {name}")
        duration = source["duration_seconds"]
    if follow_input is None:
        follow_input = source.get("follow_input", False) if source else False
    if type(follow_input) is not bool:
        raise ValueError("follow_input must be boolean")
    if follow_input and (object_controller is not None or dual_arm_controller is not None):
        raise ValueError("known-input baseline cannot be combined with custom controller imports")
    if source is not None and follow_delay_ms is not None:
        raise ValueError('replay preserves recorded follow delay; do not override it')
    follow_delay_ms = (source.get('follow_delay_ms', 0.) if source else 0.) if follow_delay_ms is None else follow_delay_ms
    if follow_delay_ms and not follow_input:
        raise ValueError('follow delay requires --follow-input')
    if source is not None:
        if scene_config is not None:
            raise ValueError("replay preserves its recorded scene configuration")
        scene_config = ROOT / source['scene_config'] if source.get('scene_config') else None
        for name, supplied in (("object_controller", object_controller), ("dual_arm_controller", dual_arm_controller)):
            if supplied is not None and supplied != source.get(name):
                raise ValueError("replay preserves recorded controller imports")
        object_controller = source.get('object_controller')
        dual_arm_controller = source.get('dual_arm_controller')
    config_path = CONFIG if scene_config is None else Path(scene_config).resolve()
    config = json.loads(config_path.read_text())
    follow_settings = config.get('human_follow', {})
    if follow_settings and not follow_input and not (object_controller and dual_arm_controller):
        raise ValueError("candidate preview requires --follow-input or both custom controllers")
    if source is not None and any(x is not None for x in (speed_deg_s, accel_deg_s2, limit_deg)):
        raise ValueError("replay preserves recorded motion parameters; do not override them")
    if source is None:
        operating_bound = follow_settings.get('angle_limit_deg', 60.) if follow_input else 10.
        limit_deg = operating_bound if limit_deg is None else limit_deg
        if isinstance(limit_deg, bool) or not np.isfinite(limit_deg) or not 0 < limit_deg <= operating_bound:
            raise ValueError(f"angle limit must be within (0, {operating_bound}] deg for this controller")
    hand = (HandTargets(speed_deg_s=15. if speed_deg_s is None else speed_deg_s,
                        limit_deg=limit_deg,
                        accel_deg_s2=60. if accel_deg_s2 is None else accel_deg_s2)
            if source is None else HandTargets(**source['target_parameters']))
    events = [] if source is None else json.loads((replay / "events.json").read_text())
    if source is not None:
        if not events or events[0] != hand.events[0]:
            raise ValueError("replay must start with the canonical zero-offset HOLD")
        events = events[1:]
    if mode == "demo":
        # Deliberate mid-motion stop, reverse, then hold. No robot trajectory is prescribed.
        events = [dict(timestamp=1., mode="ROTATE_POS", goal_offset_deg=10.),
                  dict(timestamp=1.5, mode="HOLD"),
                  dict(timestamp=3., mode="ROTATE_NEG", goal_offset_deg=-10.),
                  dict(timestamp=4., mode="HOLD")]
    for event in events:
        hand.add(**event)
    from phase1b.human_follow import DelayedHandReference
    follower = DelayedHandReference(hand, follow_delay_ms) if follow_input else None
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    session = dict(version=VERSION, status="preparing", mode=mode,
                   scene_config=str(config_path.relative_to(ROOT)) if config_path.is_relative_to(ROOT) else str(config_path),
                   follow_settings=follow_settings,
                   follow_delay_ms=follow_delay_ms,
                   follow_delay_scope='sampled human reference availability; simulation time; human and physical feedback remain current',
                   replay_of=str(replay) if source else None, follow_input=follow_input,
                   control_scope="known_input_position_follow" if follow_input else "external_or_position_hold",
                   target_parameters=dict(speed_deg_s=hand.speed, limit_deg=hand.limit, accel_deg_s2=hand.accel),
                   motion_profile="bounded_acceleration_trapezoid_or_triangle",
                   object_controller=object_controller, dual_arm_controller=dual_arm_controller,
                   tracking_validated=False, expert_labels_validated=False)
    write_json(output / "session.json", session)
    world = None
    try:
        world = HumanWorld(config_path=config_path)
        saved = world.prepare()
        if source is not None:
            if source["model_hash"] != world.config["model_sha256"] or source["config_hash"] != sha(config_path):
                raise ValueError("replay model/config differs")
            with np.load(replay / "initial_state.npz", allow_pickle=False) as checkpoint:
                saved = dict(checkpoint)
        # Original and replay both enter through the same checkpoint restoration.
        world.restore(saved)
        np.savez_compressed(output / "initial_state.npz", **saved)
        world.attach(hand, "manual_keyboard" if mode == "manual" else
                     source["input_source"] if source else "scripted_input_test")
        session.update(input_source=world.input_source, model_hash=world.config["model_sha256"],
                       config_hash=sha(config_path),
                       hand_target_origin_world_m=world.fixed_target.tolist(),
                       rotation_pivot_world_m=world.pivot.tolist(),
                       initial_panel_pitch_rad=world.initial_pitch,
                       human_stiffness_n_m=world.p["kh"], human_damping_ns_m=world.p["dh"],
                       human_force_limit_n=world.p["human_force_limit"],
                       target_speed_limit_m_s=world.p["human_target_speed_limit"],
                       source_sha256=sha(Path(__file__)))
        components = dict(policy=PassivePolicy(), object_controller=load_controller(object_controller),
                          dual_arm_controller=load_controller(dual_arm_controller))
        if follow_input:
            from phase1b.human_follow import KnownInputPolicy, KnownInputObjectReference, PositionFollowArms
            initial = world.snapshot()[0]
            initial_pose = np.r_[initial["object_position"], initial["object_orientation"]]
            components = dict(policy=KnownInputPolicy(follower),
                              object_controller=KnownInputObjectReference(follower, initial_pose),
                              dual_arm_controller=PositionFollowArms(world.m, world.qtarget, initial_pose,
                                  joint_reserve_deg=follow_settings.get('joint_reserve_deg', 0.)))
        session['clearance_checks'] = 0
        pending = SimpleQueue()
        session["raw_inputs"] = []
        stopped = False
        wall_start = time.monotonic()

        def receive(key):
            if mode == "manual" and key in tuple(map(ord, "JKLQUO")):
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
                direction = 1 if key in (ord("L"), ord("O")) else -1
                step_deg = 1. if key in (ord("U"), ord("O")) else 5.
                goal = float(np.clip(angle + direction * step_deg, -hand.limit, hand.limit))
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
            viewer_context = mujoco.viewer.launch_passive(display_model, display_data, key_callback=receive,
                                                         show_left_ui=False, show_right_ui=False)
        with viewer_context as viewer:
            if viewer is not None:
                viewer.set_texts((mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                  (("Known-input FOLLOW | no intent inference" if follow_input else
                                    "Custom controller | assistance unverified" if object_controller or dual_arm_controller else
                                    "Human input pilot | Robot assistance: OFF")
                                   + f" | range +/-{hand.limit:g} deg"
                                   + (f" | follow delay {follow_delay_ms:g} ms" if follow_input else "")
                                   + (" | BASE CANDIDATE" if follow_settings else "")), ""))
                viewer.cam.lookat[:] = [0, .3, .40]
                viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 1.8, 90, -25
                print("J: negative goal | L: positive goal | K: brake to hold | Q/close: save and exit", flush=True)
            tactile_baseline = world.snapshot()[0]
            pace_start = time.monotonic()

            def guard(w):
                physical_guard(w)
                session['clearance_checks'] = w.clearance_checks
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
                        # Absolute normal-force distribution and signed change share the scene clock.
                        from PIL import Image
                        display_truth = dict(w.truth)
                        if follower is not None and follower.records:
                            display_truth['follower_timing'] = follower.records[-1]
                        pixels = tactile_image(w.snapshot()[0], tactile_baseline, display_truth)
                        width = min(1000, viewer.viewport.width, round(viewer.viewport.height * .48 * 760 / 382))
                        height = round(382 * width / 760)
                        pixels = np.asarray(Image.fromarray(pixels).resize((width, height)))
                        viewer.set_images((mujoco.MjrRect(max(0, (viewer.viewport.width - width) // 2), 0, width, height), pixels))
                        viewer.sync()
                    time.sleep(max(0, pace_start + w.tick * .002 - time.monotonic()))

            verification = run_episode(output / "episode", "B", duration, world=world,
                **components, step_guard=guard, history_length=1,  # Pilot policies use only the current clock; full observations are still logged.
                stop_requested=lambda: stopped or (viewer is not None and not viewer.is_running()),
                episode_context=dict(task_id="HUMAN_INPUT_PILOT", input_contract=VERSION,
                                     input_source=world.input_source, formal_demonstration=False,
                                     follow_delay_ms=follow_delay_ms,
                                     known_human_input_baseline=follow_input))
        data, meta = read_episode(output / "episode")
        if follower is not None:
            timing = stack(follower.records)
            np.testing.assert_allclose(timing['decision_timestamp'], data['observations']['timestamp'], atol=1e-12)
            timing['arm_command_timestamps'] = data['controller']['command_timestamp'].copy()
            timing['arm_reference_age_s'] = timing['arm_command_timestamps']-timing['source_sample_timestamp'][:,None]
            np.savez_compressed(output/'follower_timing.npz', **timing)
            session['follower_timing_complete'] = True
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
        if follower is not None and follower.records and not session.get('follower_timing_complete'):
            np.savez_compressed(output/'partial_follower_timing.npz', **stack(follower.records))
        write_json(output / "events.json", hand.events)
        session["replay_hashes"] = {name: sha(output / name) for name in ("events.json", "initial_state.npz") if (output / name).exists()}
        write_json(output / "session.json", session)
    return session


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--mode", choices=("demo", "manual", "replay"), default="demo")
    parser.add_argument("--replay", type=Path)
    parser.add_argument("--scene-config", type=Path, help="opt-in scene configuration; replay inherits its recorded scene")
    parser.add_argument("--duration", type=float, default=10.)
    parser.add_argument("--viewer", dest="show", action="store_true")
    parser.add_argument("--speed-deg-s", type=float, help="default 15 deg/s; preserved in replay")
    parser.add_argument("--accel-deg-s2", type=float, help="default 60 deg/s^2; preserved in replay")
    parser.add_argument("--limit-deg", type=float,
                        help="symmetric bound: default/max 90 for base candidate, 60 for original following, otherwise 10")
    parser.add_argument("--follow-input", action=argparse.BooleanOptionalAction, default=None,
                        help="known-input IK motion baseline; replay inherits source unless explicitly overridden")
    parser.add_argument("--follow-delay-ms", type=float,
                        help="sampled-reference delay in simulation time; multiple of 10 ms, default 0; replay inherits source")
    parser.add_argument("--object-controller", help="module:Class; same public controller interface")
    parser.add_argument("--dual-arm-controller", help="module:Class; same public controller interface")
    args = parser.parse_args()
    print(json.dumps(run_human(**vars(args)), indent=2))


if __name__ == "__main__":
    main()
