import numpy as np

from phase1b.interfaces import (
    ObjectCommand,
    ArmCommand,
)

from phase1b.kinematics import (
    DualPandaKinematics,
)


class ObjectController:
    def __init__(self):
        self.pose_ref = None

    def step(self, decision, observation, dt):
        obs = observation.observation

        current_pose = np.concatenate(
            [
                obs["object_position"],
                obs["object_orientation"],
            ]
        )

        if self.pose_ref is None:
            self.pose_ref = current_pose.copy()

        assist_wrench = np.array(
            [
                0.0,
                0.0,
                0.0,
                0.0,
                decision.assist_torque_y_nm,
                0.0,
            ],
            dtype=float,
        )

        return ObjectCommand(
            timestamp=decision.timestamp,
            mode=decision.mode,
            pose_ref_world=self.pose_ref.copy(),
            twist_ref_world=np.zeros(6, dtype=float),
            assist_wrench_world=assist_wrench,
            valid=decision.valid,
        )


class DualArmController:
    def __init__(self):
        self.kinematics = DualPandaKinematics()

        self.initialized = False

        self.initial_left_position = None
        self.initial_right_position = None
        self.initial_object_position = None

        self.counter = 0

        # ---------------------------------------------------------
        # Torque → Cartesian displacement mapping
        #
        # Initial conservative gain.
        #
        # Units:
        #   meters / Nm
        #
        # tau = 0.01 Nm
        # gives roughly dz = 1 mm before saturation.
        # ---------------------------------------------------------
        self.torque_to_offset_gain = 0.10

        # Maximum allowed vertical offset per gripper
        self.max_offset = 0.0010  # 1 mm

        # Damped IK parameter
        self.ik_damping = 1e-3

    def step(self, command, observation):
        obs = observation.observation

        q_measured = obs["arm_joint_position"]

        left_q = q_measured[0].copy()
        right_q = q_measured[1].copy()

        # ---------------------------------------------------------
        # Nominal joint targets are the baseline support posture.
        # Do NOT replace these with measured joint positions.
        # ---------------------------------------------------------
        q_nominal = observation.nominal_arm_position.copy()

        # Update isolated kinematic model
        self.kinematics.set_arm_state(
            left_q,
            right_q,
        )

        fk = self.kinematics.forward_kinematics(
            left_q,
            right_q,
        )

        left_pos = fk["left_position"]
        right_pos = fk["right_position"]

        # ---------------------------------------------------------
        # Initialization
        # ---------------------------------------------------------
        if not self.initialized:
            self.initial_left_position = left_pos.copy()
            self.initial_right_position = right_pos.copy()
            self.initial_object_position = (
                obs["object_position"].copy()
            )

            print("\n========== ASSISTANCE CONTROLLER ==========")
            print("Initial left EE:")
            print(self.initial_left_position)

            print("\nInitial right EE:")
            print(self.initial_right_position)

            print("\nInitial panel position:")
            print(self.initial_object_position)

            print(
                "\nTorque-to-offset gain:",
                self.torque_to_offset_gain,
                "m/Nm",
            )

            print(
                "Maximum offset:",
                self.max_offset,
                "m",
            )

            print(
                "===========================================\n"
            )

            self.initialized = True

        # ---------------------------------------------------------
        # Requested object assistance torque
        #
        # Wrench ordering:
        #
        # [Fx, Fy, Fz, Tx, Ty, Tz]
        #
        # index 4 = Ty
        # ---------------------------------------------------------
        requested_tau_y = float(
            command.assist_wrench_world[4]
        )

        # ---------------------------------------------------------
        # Map requested torque to symmetric vertical displacement.
        #
        # Positive Ty:
        #
        # left  +Z
        # right -Z
        #
        # Negative Ty reverses automatically.
        # ---------------------------------------------------------
        dz = (
            self.torque_to_offset_gain
            * requested_tau_y
        )

        # Safety saturation
        dz = np.clip(
            dz,
            -self.max_offset,
            +self.max_offset,
        )

        delta_left = np.array(
            [
                0.0,
                0.0,
                +dz,
            ],
            dtype=float,
        )

        delta_right = np.array(
            [
                0.0,
                0.0,
                -dz,
            ],
            dtype=float,
        )

        # ---------------------------------------------------------
        # Jacobians
        # ---------------------------------------------------------
        J_left = self.kinematics.jacobian("left")
        J_right = self.kinematics.jacobian("right")

        # Position portion only
        J_left_pos = J_left[:3, :]
        J_right_pos = J_right[:3, :]

        damping = self.ik_damping

        left_pinv = (
            J_left_pos.T
            @ np.linalg.inv(
                J_left_pos @ J_left_pos.T
                + (damping ** 2) * np.eye(3)
            )
        )

        right_pinv = (
            J_right_pos.T
            @ np.linalg.inv(
                J_right_pos @ J_right_pos.T
                + (damping ** 2) * np.eye(3)
            )
        )

        # ---------------------------------------------------------
        # Compute joint offsets corresponding to dz
        # ---------------------------------------------------------
        left_joint_offset = (
            left_pinv @ delta_left
        )

        right_joint_offset = (
            right_pinv @ delta_right
        )

        # ---------------------------------------------------------
        # Final command:
        #
        # baseline support
        # +
        # incremental assistance
        # ---------------------------------------------------------
        arm_target = q_nominal.copy()

        arm_target[0] += left_joint_offset
        arm_target[1] += right_joint_offset

        # ---------------------------------------------------------
        # Diagnostics
        # ---------------------------------------------------------
        self.counter += 1

        if self.counter % 500 == 0:
            object_position = obs["object_position"]
            object_wrench = obs["object_contact_wrench"]

            left_delta = (
                left_pos
                - self.initial_left_position
            )

            right_delta = (
                right_pos
                - self.initial_right_position
            )

            object_delta = (
                object_position
                - self.initial_object_position
            )

            print("\n--- ASSISTANCE STATUS ---")

            print(
                "Requested Ty:",
                requested_tau_y,
                "Nm",
            )

            print(
                "Commanded dz:",
                dz,
                "m",
            )

            print("\nLeft EE delta:")
            print(left_delta)

            print("\nRight EE delta:")
            print(right_delta)

            print("\nPanel position change:")
            print(object_delta)

            print("\nMeasured object wrench:")
            print(object_wrench)

            print(
                "\nMeasured Ty:",
                object_wrench[4],
                "Nm",
            )

            print(
                "-------------------------\n"
            )

        # ---------------------------------------------------------
        # Mark assistance as executed when a nonzero reference
        # actually reaches this controller.
        # ---------------------------------------------------------
        assistance_active = (
            abs(requested_tau_y) > 1e-9
        )

        return ArmCommand(
            arm_position_target=arm_target,

            pad_force_command=
                observation.nominal_pad_force.copy(),

            controller_mode="incremental_pitch_assistance",

            assist_reference_executed=assistance_active,
        )