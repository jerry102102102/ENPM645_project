"""Known-input position-following pilot, not an Expert or torque controller.

Only the test policy/object reference sees the causal human target source.
The arm adapter receives ObjectCommand and public state through the normal chain.
"""

import copy
from collections import deque

import mujoco
import numpy as np

from phase1b.geometry import rotate_y, solve
from phase1b.interfaces import ArmCommand, AssistanceDecision, ObjectCommand, current, public_input


def matrix(quaternion):
    result = np.zeros(9)
    mujoco.mju_quat2Mat(result, quaternion)
    return result.reshape(3, 3)


class DelayedHandReference:
    """Delay sampled human references in simulation time; never delay the human itself."""

    def __init__(self, hand, delay_ms=0.):
        if (isinstance(delay_ms, bool) or not np.isfinite(delay_ms) or delay_ms < 0
                or not np.isclose(delay_ms / 10, round(delay_ms / 10), atol=1e-10, rtol=0)):
            raise ValueError('follow delay must be nonnegative, finite, and a multiple of 10 ms')
        self.hand, self.delay_ticks = hand, round(delay_ms / 10)
        self.pending, self.records = deque(), []
        self.last_tick = -1
        self.delivered = None
        self.value = (0., 0., dict(mode='HOLD', timestamp=0., goal_offset_deg=0.))

    def sample(self, timestamp):
        if not np.isfinite(timestamp):
            raise ValueError('finite follower timestamp required')
        tick = round(timestamp / .01)
        if timestamp < 0 or not np.isclose(timestamp, tick*.01, atol=1e-10, rtol=0) or tick < self.last_tick:
            raise ValueError('follower samples require monotonic 100 Hz timestamps')
        if tick == self.last_tick:
            return self.value
        if tick != self.last_tick + 1:
            raise ValueError('follower reference sample skipped a 100 Hz tick')
        angle, speed, event = self.hand.sample(timestamp)
        self.pending.append((tick, float(angle), float(speed), dict(event)))
        while self.pending and self.pending[0][0] <= tick-self.delay_ticks:
            self.delivered = self.pending.popleft()
        ready = self.delivered is not None
        if ready:
            source_tick, x, v, source_event = self.delivered
            self.value = (x, v, dict(source_event))
        source_time = self.delivered[0]*.01 if ready else np.nan
        self.records.append(dict(
            decision_timestamp=timestamp, input_sample_timestamp=timestamp,
            source_sample_timestamp=source_time, reference_available=ready,
            configured_delay_s=self.delay_ticks*.01,
            reference_age_s=timestamp-source_time,
            human_event_timestamp=float(event['timestamp']),
            source_event_timestamp=float(self.value[2]['timestamp']) if ready else np.nan,
            human_target_angle_deg=float(angle), follower_target_angle_deg=self.value[0],
            human_target_velocity_deg_s=float(speed), follower_target_velocity_deg_s=self.value[1],
            human_mode=event['mode'], follower_mode=self.value[2]['mode']))
        self.last_tick = tick
        return self.value


class KnownInputPolicy:
    def __init__(self, hand):
        self.hand = hand

    def predict(self, history):
        t = float(history.timestamps[-1])
        _, _, event = self.hand.sample(t)
        return AssistanceDecision(t, event["mode"], 0., None, "scripted_test", True)


class KnownInputObjectReference:
    def __init__(self, hand, initial_pose):
        self.hand = hand
        self.initial = np.array(initial_pose, copy=True)

    def step(self, decision, observation, dt):
        observation = public_input(observation)
        current(decision.timestamp, observation.timestamp, 0)
        if not np.isclose(dt, .01):
            raise ValueError("100 Hz reference required")
        angle, speed, event = self.hand.sample(observation.timestamp)
        if decision.mode != event["mode"] or decision.assist_torque_y_nm != 0:
            raise ValueError("known-input motion test requires matching mode and zero torque request")
        R = rotate_y(np.deg2rad(angle)) @ matrix(self.initial[3:])
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, R.ravel())
        return ObjectCommand(observation.timestamp, decision.mode,
                             np.r_[self.initial[:3], quat],
                             np.array([0., np.deg2rad(speed), 0., 0., 0., 0.]),
                             np.zeros(6))


class PositionFollowArms:
    """Incremental IK on a separate model; execute only bounded position-servo commands."""

    max_joint_command_rate_rad_s = .5

    def __init__(self, model, nominal_q, initial_pose, joint_reserve_deg=0.):
        if not np.isfinite(joint_reserve_deg) or not 0 <= joint_reserve_deg < 30:
            raise ValueError("joint reserve must be within [0,30) degrees")
        self.m = copy.copy(model)
        self.d = mujoco.MjData(self.m)
        joints = np.array([[self.m.joint(f"{s}_joint{i}").id for i in range(1, 8)]
                           for s in ("left", "right")])
        self.m.jnt_range[joints, 0] += np.deg2rad(joint_reserve_deg)
        self.m.jnt_range[joints, 1] -= np.deg2rad(joint_reserve_deg)
        self.initial = np.array(initial_pose, copy=True)
        self.qa = np.array([[self.m.jnt_qposadr[self.m.joint(f"{s}_joint{i}").id]
                            for i in range(1, 8)] for s in ("left", "right")])
        self.d.qpos[self.qa] = nominal_q
        mujoco.mj_forward(self.m, self.d)
        sites = [self.m.site(s + "_ee").id for s in ("left", "right")]
        self.positions = self.d.site_xpos[sites].copy()
        self.rotations = self.d.site_xmat[sites].reshape(2, 3, 3).copy()
        self.sent = np.array(nominal_q, copy=True)
        self.target = self.sent.copy()
        self.reference_time = None

    def step(self, command, observation):
        observation = public_input(observation)
        current(command.timestamp, observation.timestamp, .008)
        if not command.valid or command.mode == "UNCERTAIN" or np.any(command.assist_wrench_world):
            raise ValueError("position-follow pilot accepts valid motion references with zero assistance wrench")
        if command.timestamp != self.reference_time:
            delta = matrix(command.pose_ref_world[3:]) @ matrix(self.initial[3:]).T
            for arm, side in enumerate(("left", "right")):
                position = command.pose_ref_world[:3] + delta @ (self.positions[arm] - self.initial[:3])
                result = solve(self.m, self.d, side, position, delta @ self.rotations[arm], self.target[arm])
                if result is None:
                    raise RuntimeError(f"known-input IK failed for {side} at {command.timestamp}")
            self.target = self.d.qpos[self.qa].copy()
            self.reference_time = command.timestamp
        step = self.max_joint_command_rate_rad_s * observation.control_dt
        self.sent += np.clip(self.target - self.sent, -step, step)
        return ArmCommand(self.sent.copy(), observation.nominal_pad_force.copy(),
                          "known_input_position_follow", False)


class InterArmClearance:
    """Conservative convex-geometry separation bound, not native distance values."""

    def __init__(self, model):
        import itertools
        self.m = model
        groups = []
        for side in ('left', 'right'):
            root = model.body(side+'_link0').id
            group = []
            for g in range(model.ngeom):
                b = model.geom_bodyid[g]
                while b and b != root:
                    b = model.body_parentid[b]
                if b == root and (model.geom_contype[g] or model.geom_conaffinity[g]):
                    group.append(g)
            groups.append(group)
        self.pairs = np.array([(a,b) for a in groups[0] for b in groups[1]
                              if (model.geom_contype[a]&model.geom_conaffinity[b])
                              or (model.geom_contype[b]&model.geom_conaffinity[a])])
        self.vertices = {}
        for g in sum(groups, []):
            if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
                mesh = model.geom_dataid[g]
                start, n = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
                v = model.mesh_vert[start:start+n].astype(float)
            elif model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
                v = np.array(list(itertools.product((-1,1), repeat=3)))*model.geom_size[g]
            else:
                raise ValueError('clearance bound supports the current mesh/box arm geometry only')
            if np.linalg.norm(v,axis=1).max() > model.geom_rbound[g]+1e-7:
                raise ValueError('invalid bounding sphere')
            self.vertices[g] = v

    def lower_bound(self, data, cutoff=.1):
        """Return a lower bound capped at cutoff; caller must stop if uncertified."""
        m, pairs, bound = self.m, self.pairs, cutoff
        lower = (np.linalg.norm(data.geom_xpos[pairs[:,0]]-data.geom_xpos[pairs[:,1]],axis=1)
                 - m.geom_rbound[pairs[:,0]] - m.geom_rbound[pairs[:,1]])
        for a,b in pairs[lower < cutoff]:
            witness = np.zeros(6)
            # Some engine versions return inconsistent scalar distances. Use only a
            # proposed direction; the vertex projections independently certify its gap.
            mujoco.mj_geomDistance(m,data,int(a),int(b),max(.5,cutoff),witness)
            directions = np.vstack([witness[3:]-witness[:3],data.geom_xpos[b]-data.geom_xpos[a],np.eye(3)])
            directions = directions[np.linalg.norm(directions,axis=1)>1e-12]
            directions /= np.linalg.norm(directions,axis=1)[:,None]
            pa,pb = [(self.vertices[g]@data.geom_xmat[g].reshape(3,3).T+data.geom_xpos[g])@directions.T for g in (a,b)]
            gap = np.maximum(pb.min(axis=0)-pa.max(axis=0),pa.min(axis=0)-pb.max(axis=0)).max()-1e-8
            bound = min(bound,max(0.,float(gap)))
        return bound


def scan_pitch(output, step_deg=.5, extent_deg=90.):
    """Local IK branch + enabled collision pairs at sampled poses; not a dynamic limit."""
    import json
    from pathlib import Path
    from phase1b.benchmark import BenchmarkWorld
    from phase1b.hardware import sha
    from phase1b.simulation import CONFIG

    if not np.isfinite(step_deg) or not np.isfinite(extent_deg) or not 0 < step_deg <= extent_deg <= 180:
        raise ValueError("finite positive step <= extent <= 180 required")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    world = BenchmarkWorld()
    world.restore(world.prepare())
    initial, _ = world.snapshot()
    pose = np.r_[initial['object_position'], initial['object_orientation']]
    report = dict(scope='Sampled local IK branch and enabled MJCF collision pairs; not a global workspace proof or physical support limit.',
                  axis='world +Y', pivot_world_m=pose[:3].tolist(), step_deg=step_deg,
                  model_sha256=world.config['model_sha256'], config_sha256=sha(CONFIG),
                  source_sha256=sha(Path(__file__)), branches={})
    for direction in (1, -1):
        planner = PositionFollowArms(world.m, world.qtarget, pose)
        planner.d.qpos[:] = world.d.qpos
        planner.d.qpos[planner.qa] = world.qtarget
        seed = world.qtarget.copy()
        pad_geoms = sum(world.pad_g, [])
        rows = []
        for magnitude in np.arange(0, extent_deg + 1e-9, step_deg):
            angle = float(direction * magnitude)
            delta = rotate_y(np.deg2rad(angle))
            row = dict(angle_offset_deg=angle, passed=False, ik={})
            for arm, side in enumerate(('left', 'right')):
                result = solve(planner.m, planner.d, side,
                               pose[:3] + delta @ (planner.positions[arm] - pose[:3]),
                               delta @ planner.rotations[arm], seed[arm])
                row['ik'][side] = result
                if result is None:
                    row['reason'] = f'{side}_local_ik_did_not_converge'
                    break
            else:
                seed = planner.d.qpos[planner.qa].copy()
                R = delta @ matrix(pose[3:])
                quat = np.zeros(4)
                mujoco.mju_mat2Quat(quat, R.ravel())
                address = planner.m.jnt_qposadr[planner.m.joint('panel_free').id]
                planner.d.qpos[address:address+3] = pose[:3] - R @ planner.m.body_ipos[world.panel]
                planner.d.qpos[address+3:address+7] = quat
                mujoco.mj_forward(planner.m, planner.d)
                collisions = []
                for c in planner.d.contact:
                    expected = world.pg in (c.geom1, c.geom2) and (c.geom1 in pad_geoms or c.geom2 in pad_geoms)
                    if not expected and c.dist < -1e-5:
                        collisions.append(dict(geom1_id=int(c.geom1), geom2_id=int(c.geom2),
                                               body1=planner.m.body(planner.m.geom_bodyid[c.geom1]).name,
                                               body2=planner.m.body(planner.m.geom_bodyid[c.geom2]).name,
                                               geom1=planner.m.geom(c.geom1).name,
                                               geom2=planner.m.geom(c.geom2).name,
                                               penetration_m=float(-c.dist)))
                margin = np.minimum(seed - planner.m.jnt_range[world.arm_joints, 0],
                                    planner.m.jnt_range[world.arm_joints, 1] - seed)
                k = np.unravel_index(np.argmin(margin), margin.shape)
                row.update(joint_positions_rad=seed.tolist(), min_joint_margin_rad=float(margin.min()),
                           closest_joint=world.config['arm_joint_names'][('left','right')[k[0]]][k[1]],
                           collisions=collisions, passed=bool(not collisions and margin.min() > 0),
                           reason='unexpected_collision' if collisions else 'joint_limit' if margin.min() <= 0 else 'sample_passed')
            rows.append(row)
            if not row['passed']:
                break
        passed = [r['angle_offset_deg'] for r in rows if r['passed']]
        report['branches']['positive' if direction > 0 else 'negative'] = dict(
            last_passed_offset_deg=passed[-1] if passed else None,
            first_failed_offset_deg=rows[-1]['angle_offset_deg'] if not rows[-1]['passed'] else None,
            stop_reason=rows[-1]['reason'], samples=rows)
    (output/'pitch_scan.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


def probe_pitch(output, extent_deg=90., speed_deg_s=15., accel_deg_s2=60.):
    """Directional physical probes; failures are preserved and never promoted to GO."""
    import json
    from pathlib import Path
    from phase1b.human import HandTargets, HumanWorld
    from phase1b.benchmark import physical_guard
    from phase1b.pipeline import run_episode

    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    if not np.isfinite(extent_deg) or not 0 < extent_deg <= 180:
        raise ValueError('extent must be within (0,180]')
    reports = {}
    for sign, name in ((1,'positive'), (-1,'negative')):
        world = HumanWorld()
        world.restore(world.prepare())
        hand = HandTargets(speed_deg_s=speed_deg_s, limit_deg=extent_deg, accel_deg_s2=accel_deg_s2)
        hand.add(1., 'ROTATE_POS' if sign > 0 else 'ROTATE_NEG', sign*extent_deg)
        world.attach(hand, 'scripted_pitch_range_probe')
        o, _ = world.snapshot()
        pose = np.r_[o['object_position'], o['object_orientation']]
        R0 = matrix(pose[3:])
        report = dict(passed=False, target_extent_deg=sign*extent_deg,
                      speed_deg_s=speed_deg_s, accel_deg_s2=accel_deg_s2,
                      last_guard_passed_time_s=0., last_guard_passed_target_deg=0.,
                      last_guard_passed_actual_deg=0., last_guard_passed_max_command_gap_rad=0.)
        reports[name] = report
        controller = PositionFollowArms(world.m, world.qtarget, pose)

        def guard(w):
            physical_guard(w)
            rel = w.d.xmat[w.panel].reshape(3,3) @ R0.T
            report.update(last_guard_passed_time_s=w.tick*.002,
                          last_guard_passed_target_deg=hand.sample(w.tick*.002)[0],
                          last_guard_passed_actual_deg=float(np.rad2deg(np.arctan2(rel[0,2], rel[0,0]))),
                          last_guard_passed_max_command_gap_rad=float(abs(controller.target-controller.sent).max()))

        duration = np.ceil((3. + extent_deg/speed_deg_s + 2*speed_deg_s/accel_deg_s2)*100)/100
        try:
            result = run_episode(output/name, 'B', duration, world=world, step_guard=guard,
                policy=KnownInputPolicy(hand), object_controller=KnownInputObjectReference(hand, pose),
                dual_arm_controller=controller, history_length=1,
                episode_context=dict(task_id='PITCH_RANGE_PROBE', known_human_input_baseline=True))
            report.update(passed=bool(result['physical_sample_checks_passed']), verification=result,
                          stop_reason='completed' if result['physical_sample_checks_passed'] else 'physical_checks_failed')
        except RuntimeError as error:
            report.update(stop_reason=str(error), failure_time_s=world.tick*.002,
                          failed_target_deg=hand.sample(world.tick*.002)[0])
        (output/'physical_probe.json').write_text(json.dumps(reports, indent=2)+'\n')
    return reports


if __name__ == '__main__':
    import argparse
    import json
    parser = argparse.ArgumentParser(description='Sample the frozen two-arm pitch path; not a physical safety certification.')
    parser.add_argument('--output', required=True)
    parser.add_argument('--step-deg', type=float, default=.5)
    parser.add_argument('--extent-deg', type=float, default=90.)
    parser.add_argument('--physics', action='store_true', help='probe physical support instead of sampled IK')
    args = parser.parse_args()
    if args.physics:
        print(json.dumps(probe_pitch(args.output, args.extent_deg), indent=2))
    else:
        result = scan_pitch(args.output, args.step_deg, args.extent_deg)
        print(json.dumps({k:{a:b for a,b in v.items() if a != 'samples'} for k,v in result['branches'].items()}, indent=2))
