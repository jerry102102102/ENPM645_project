from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "phase1b/models/dual_panda_v1.xml"


class DualPandaKinematics:
    """
    Isolated MuJoCo model used only for kinematics calculations.

    This does not access the benchmark's live MjData.
    It reconstructs the robot state from observed joint positions.
    """

    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        self.data = mujoco.MjData(self.model)

        self.left_site_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_SITE,
            "left_ee",
        )

        self.right_site_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_SITE,
            "right_ee",
        )

        self.left_joint_names = [
            f"left_joint{i}" for i in range(1, 8)
        ]

        self.right_joint_names = [
            f"right_joint{i}" for i in range(1, 8)
        ]

        self.left_qpos_ids = []
        self.right_qpos_ids = []

        self.left_dof_ids = []
        self.right_dof_ids = []

        for name in self.left_joint_names:
            joint_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                name,
            )

            self.left_qpos_ids.append(
                self.model.jnt_qposadr[joint_id]
            )

            self.left_dof_ids.append(
                self.model.jnt_dofadr[joint_id]
            )

        for name in self.right_joint_names:
            joint_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                name,
            )

            self.right_qpos_ids.append(
                self.model.jnt_qposadr[joint_id]
            )

            self.right_dof_ids.append(
                self.model.jnt_dofadr[joint_id]
            )

        self.left_qpos_ids = np.array(
            self.left_qpos_ids,
            dtype=int,
        )

        self.right_qpos_ids = np.array(
            self.right_qpos_ids,
            dtype=int,
        )

        self.left_dof_ids = np.array(
            self.left_dof_ids,
            dtype=int,
        )

        self.right_dof_ids = np.array(
            self.right_dof_ids,
            dtype=int,
        )

    def set_arm_state(self, left_q, right_q):
        """
        Set the 7 arm joints for each Panda.

        left_q, right_q: shape (7,)
        """

        self.data.qpos[self.left_qpos_ids] = left_q
        self.data.qpos[self.right_qpos_ids] = right_q

        mujoco.mj_forward(self.model, self.data)

    def site_pose(self, site_id):
        """
        Return:
            position: shape (3,)
            rotation matrix: shape (3,3)
        """

        position = self.data.site_xpos[site_id].copy()

        rotation = self.data.site_xmat[
            site_id
        ].reshape(3, 3).copy()

        return position, rotation

    def forward_kinematics(self, left_q, right_q):
        """
        Returns FK for both end effectors.
        """

        self.set_arm_state(left_q, right_q)

        left_pos, left_rot = self.site_pose(
            self.left_site_id
        )

        right_pos, right_rot = self.site_pose(
            self.right_site_id
        )

        return {
            "left_position": left_pos,
            "left_rotation": left_rot,
            "right_position": right_pos,
            "right_rotation": right_rot,
        }

    def jacobian(self, arm):
        """
        Return the 6x7 geometric Jacobian.

        Ordering:
            first 3 rows = linear velocity
            last 3 rows = angular velocity

        This is MuJoCo's native Jacobian ordering.
        """

        if arm == "left":
            site_id = self.left_site_id
            dof_ids = self.left_dof_ids

        elif arm == "right":
            site_id = self.right_site_id
            dof_ids = self.right_dof_ids

        else:
            raise ValueError("arm must be 'left' or 'right'")

        jac_pos = np.zeros(
            (3, self.model.nv),
            dtype=float,
        )

        jac_rot = np.zeros(
            (3, self.model.nv),
            dtype=float,
        )

        mujoco.mj_jacSite(
            self.model,
            self.data,
            jac_pos,
            jac_rot,
            site_id,
        )

        J = np.vstack([
            jac_pos[:, dof_ids],
            jac_rot[:, dof_ids],
        ])

        return J


def damped_pseudoinverse(J, damping=1e-3):
    """
    Damped least-squares pseudoinverse.

    J_dls = J.T (J J.T + lambda^2 I)^-1
    """

    rows = J.shape[0]

    return J.T @ np.linalg.inv(
        J @ J.T
        + (damping ** 2) * np.eye(rows)
    )


def differential_ik_step(
    J,
    task_velocity,
    damping=1e-3,
):
    """
    Convert desired Cartesian velocity into joint velocity.

    task_velocity ordering must match J:
        [vx, vy, vz, wx, wy, wz]
    """

    J_pinv = damped_pseudoinverse(
        J,
        damping=damping,
    )

    qdot = J_pinv @ task_velocity

    return qdot