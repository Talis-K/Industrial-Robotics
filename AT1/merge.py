import spatialgeometry as geometry
import numpy as np
import swift
import time
import os
from roboticstoolbox import jtraj
from ir_support import UR3
from spatialmath import SE3, SO3
from spatialgeometry import Cuboid
from math import pi


# ---------------- Gripper Class ----------------
class Gripper:
    def __init__(self, env, robot):
        self.env = env
        self.robot = robot
        self.finger_len = 0.05
        self.finger_w = 0.01
        self.finger_t = 0.04
        self.opening = 0.08
        self.carrying_idx = None

        # Create finger geometry
        self.finger_L = Cuboid(scale=[self.finger_len, self.finger_w, self.finger_t], color=[0.2, 0.2, 0.2, 1])
        self.finger_R = Cuboid(scale=[self.finger_len, self.finger_w, self.finger_t], color=[0.2, 0.2, 0.2, 1])
        self.connector = Cuboid(scale=[self.finger_len, self.opening, self.finger_w], color=[0.2, 0.2, 0.2, 1])
        env.add(self.finger_L)
        env.add(self.finger_R)
        env.add(self.connector)

    def open(self):
        self.opening = 0.08
        self.update()

    def close(self):
        self.opening = 0.06
        self.update()

    def update(self):
        """Update gripper geometry according to robot pose."""
        T_ee = self.robot.fkine(self.robot.q)

        # Rotate gripper 180° about X so fingers point downward
        T_offset = SE3.Rx(pi) * SE3.Rz(pi/2)

        half_gap = self.opening / 2
        T_fL = T_ee * T_offset * SE3(0, half_gap + 0.5 * self.finger_w, -self.finger_t / 2)
        T_fR = T_ee * T_offset * SE3(0, -half_gap - 0.5 * self.finger_w, -self.finger_t / 2)
        T_conn = T_ee * T_offset * SE3(0, 0, 0)

        self.finger_L.T = T_fL
        self.finger_R.T = T_fR
        self.connector.T = T_conn

    def update_with_payload(self, bricks):
        self.update()
        if self.carrying_idx is not None:
            T_ee = self.robot.fkine(self.robot.q)
            T_offset = SE3.Rx(pi)
            # Adjusted brick pose to center it between gripper fingers
            brick_pose = T_ee * T_offset * SE3(0, 0, -self.finger_t)
            bricks[self.carrying_idx].T = brick_pose


# ---------------- Environment Builder Class ----------------
class EnvironmentBuilder:
    def __init__(self):
        self.y_max = 0.8  # Maximum Y position for the rail carriage
        self.env = swift.Swift()
        self.env.launch(realTime=True)
        self.env.set_camera_pose([1.5, 1.3, 1.4], [0, 0, -pi/4])

        # Add environment objects
        self.ground_height = 0.01
        self.add_fences_and_ground()
        self.add_rail()
        self.safety = self.load_safety()

        # Add robot
        self.robot = UR3()
        self.robot.q = np.zeros(6)
        self.robot.base = SE3(0, 0, 0)
        self.robot.links[1].qlim = np.deg2rad([-180, 0])  # Set joint limits for the second link
        for i in range(self.robot.n):
            print(f"Joint {i} limits: lower = {np.rad2deg(self.robot.qlim[0, i]):.2f}°, upper = {np.rad2deg(self.robot.qlim[1, i]):.2f}°")
        self.robot.add_to_env(self.env)

        # Add gripper
        self.gripper = Gripper(self.env, self.robot)
        # Update gripper to set initial position
        self.gripper.update()

        # Add bricks
        self.bricks = self.load_bricks()

        # Refresh environment to show initial state
        self.env.step(0)

    def add_fences_and_ground(self):
        self.env.add(Cuboid(scale=[3, 0.05, 0.8], pose=SE3(0, 1.5, 0.4 + self.ground_height), color=[0.5, 0.9, 0.5, 0.5]))
        self.env.add(Cuboid(scale=[3, 0.05, 0.8], pose=SE3(0, -1.5, 0.4 + self.ground_height), color=[0.5, 0.9, 0.5, 0.5]))
        self.env.add(Cuboid(scale=[0.05, 3, 0.8], pose=SE3(1.5, 0, 0.4 + self.ground_height), color=[0.5, 0.9, 0.5, 0.5]))
        self.env.add(Cuboid(scale=[0.05, 3, 0.8], pose=SE3(-1.5, 0, 0.4 + self.ground_height), color=[0.5, 0.9, 0.5, 0.5]))
        self.env.add(Cuboid(scale=[3, 3, self.ground_height], pose = SE3(0, 0, 0), color=[0.9, 0.9, 0.5, 1])) 
        self.env.add(Cuboid(scale=[0.9, 0.04, 0.6], pose=SE3(-1, -1.1, 0.3), color=[0.5, 0.5, 0.9, 0.5]))

    def add_rail(self):
        self.env.add(Cuboid(scale=[0.05, 2 * self.y_max, 0.05], pose=SE3(0.1, 0, 0.025 + self.ground_height), color=[0.3, 0.3, 0.35, 1]))
        self.env.add(Cuboid(scale=[0.05, 2 * self.y_max, 0.05], pose=SE3(-0.1, 0, 0.025 + self.ground_height), color=[0.3, 0.3, 0.35, 1]))
        self.rail_carriage = Cuboid(scale=[0.15, 0.15, 0.05], pose=SE3(0.0, 0.0, 0.025), color=[1, 0.4, 0.7, 1])
        self.env.add(self.rail_carriage)

    def load_safety(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        stl_files = ["button.stl", "Fire_extinguisher.stl"]
        safety_positions = [
            SE3(-1.3, -1.4, 0.0 + self.ground_height) * SE3.Rx(pi/2), SE3(-1, -1.4, 0.0)
        ]
        safety = []
        for stl_file, pose in zip(stl_files, safety_positions):
            stl_path = os.path.join(current_dir, stl_file)
            if not os.path.exists(stl_path):
                raise FileNotFoundError(f"STL file not found: {stl_path}")
            safety_obj = geometry.Mesh(stl_path, pose=pose * SE3(0, 0, self.ground_height), scale=(0.001, 0.001, 0.001), color=(0.4, 0, 0, 1))
            self.env.add(safety_obj)
            safety.append(safety_obj)
        return safety

    def load_bricks(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        stl_path = os.path.join(current_dir, "Brick.stl")
        if not os.path.exists(stl_path):
            raise FileNotFoundError(f"STL file not found: {stl_path}")
        brick_positions = [
            SE3(-0.7, -1.2, 0.0), SE3(-0.2, 1.2, 0.0), SE3(-0.2, 0.0, 0.0), SE3(-0.2, 0.2, 0.0), SE3(-0.2, -0.2, 0.0),
            SE3(-0.3, 0.0, 0.0), SE3(-0.3, 0.2, 0.0), SE3(-0.3, -0.2, 0.0),
            SE3(-0.4, 0.0, 0.0), SE3(-0.4, 0.2, 0.0), SE3(-0.4, -0.2, 0.0)
        ]
        bricks = []
        for pose in brick_positions:
            brick = geometry.Mesh(stl_path, pose=pose * SE3(0, 0, self.ground_height), color=(0.4, 0, 0, 1))
            self.env.add(brick)
            bricks.append(brick)
        return bricks


# ---------------- Controller Class ----------------
class Controller:
    def __init__(self, env_builder: EnvironmentBuilder):
        self.env_builder = env_builder
        self.env = env_builder.env
        self.robot = env_builder.robot
        self.bricks = env_builder.bricks
        self.rail_carriage = env_builder.rail_carriage
        self.gripper = env_builder.gripper
        self.safety = env_builder.safety

        self.failed_bricks = 0  # Count of bricks that could not be reached
        # Wall target poses (only 9, so we'll handle the extra brick by skipping)
        self.wall_pose = [
            SE3(0.3, 0.15, 0.0), SE3(0.3, 0.0, 0.0), SE3(0.3, -0.15, 0.0),
            SE3(0.3, 0.15, 0.05), SE3(0.3, 0.0, 0.05), SE3(0.3, -0.15, 0.05),
            SE3(0.3, 0.15, 0.10), SE3(0.3, 0.0, 0.10), SE3(0.3, -0.15, 0.10)
        ]

    def move_carriage_to_y(self, target_y, steps=25):
        start_y = self.robot.base.t[1]
        for s in np.linspace(0, 1, steps):
            y = (1 - s) * start_y + s * target_y
            y = np.clip(y, -self.env_builder.y_max, self.env_builder.y_max)
            self.robot.base = SE3(0, y, 0)
            self.rail_carriage.T = SE3(0, y, 0.025)
            self.gripper.update_with_payload(self.bricks)
            self.env.step(0.02)
            time.sleep(0.03)

    def pick_and_place(self):
        for i in range(len(self.bricks)):
            if i-self.failed_bricks >= len(self.wall_pose):
                print(f"No wall pose for brick {i+1}, skipping")
                continue
            brick = self.bricks[i]
            brick_pose = SE3(brick.T)
            wall_pose = self.wall_pose[i-self.failed_bricks]
            print(f"Processing brick {i+1}: brick_pose={brick_pose.t}, wall_pose={wall_pose.t}")

            # Clamp target y to rail limits
            desired_y = np.clip(brick_pose.t[1], -self.env_builder.y_max, self.env_builder.y_max)
            wall_y = np.clip(wall_pose.t[1], -self.env_builder.y_max, self.env_builder.y_max)

            # Check reachability of hover and place poses before moving
            T_hover = brick_pose * SE3(0, 0, 0.05) * SE3.Ry(pi)
            if not self.check_joint_limits(None, self.robot, target_pose=T_hover, base_y=desired_y, pose_type="hover"):
                print(f"Brick {i+1} at {brick_pose.t} is out of reach from base y={desired_y}, skipping")
                continue

            T_place = wall_pose * SE3(0, 0, 0.05) * SE3.Ry(pi)
            if not self.check_joint_limits(None, self.robot, target_pose=T_place, base_y=wall_y, pose_type="place"):
                print(f"Wall pose for brick {i+1} at {wall_pose.t} is out of reach from base y={wall_y}, skipping")
                continue

            # 1) Move base close to brick
            self.move_carriage_to_y(desired_y)

            # 2) Approach hover
            ik_hover = self.robot.ikine_LM(T_hover, q0=self.robot.q, joint_limits=True)
            traj_hover = jtraj(self.robot.q, ik_hover.q, 30).q
            for q in traj_hover:
                self.robot.q = q
                self.gripper.update()
                self.env.step(0.02)
                time.sleep(0.03)

            # 3) Close gripper
            self.gripper.close()
            self.gripper.carrying_idx = i

            # 4) Lift brick slightly
            T_lift = brick_pose * SE3(0, 0, 0.1) * SE3.Ry(pi)
            ik_lift = self.robot.ikine_LM(T_lift, q0=self.robot.q, joint_limits=True)
            if not ik_lift.success:
                print(f"Cannot reach lift pose for brick {i+1}, releasing")
                self.gripper.open()
                self.gripper.carrying_idx = None
                continue
            traj_lift = jtraj(self.robot.q, ik_lift.q, 15).q
            if not self.check_joint_limits(traj_lift, self.robot, pose_type="lift"):
                print(f"Trajectory to lift for brick {i+1} violates joint limits, releasing")
                self.gripper.open()
                self.gripper.carrying_idx = None
                continue

            for q in traj_lift:
                self.robot.q = q
                self.gripper.update_with_payload(self.bricks)
                self.env.step(0.02)
                time.sleep(0.03)

            # 5) Move base to wall (at high lift to avoid collisions)
            self.move_carriage_to_y(wall_y)

            # 6) Place brick
            ik_place = self.robot.ikine_LM(T_place, q0=self.robot.q, joint_limits=True)
            traj_place = jtraj(self.robot.q, ik_place.q, 30).q
            for q in traj_place:
                self.robot.q = q
                self.gripper.update_with_payload(self.bricks)
                self.env.step(0.02)
                time.sleep(0.03)

            # 7) Release
            self.gripper.open()
            self.gripper.carrying_idx = None
            print(f"Released brick {i+1}")

            # 8) Retreat to safe height to avoid collisions in future transits
            T_retreat = SE3(wall_pose.t[0], wall_pose.t[1], 0.2) * SE3.Ry(pi)
            ik_retreat = self.robot.ikine_LM(T_retreat, q0=self.robot.q, joint_limits=True)
            if ik_retreat.success:
                traj_retreat = jtraj(self.robot.q, ik_retreat.q, 15).q
                if self.check_joint_limits(traj_retreat, self.robot, pose_type="retreat"):
                    for q in traj_retreat:
                        self.robot.q = q
                        self.gripper.update()
                        self.env.step(0.02)
                        time.sleep(0.03)
                else:
                    print(f"Trajectory to retreat for brick {i+1} violates joint limits")
            else:
                print(f"Cannot reach retreat pose for brick {i+1}")

    def check_joint_limits(self, Q, robot, target_pose=None, base_y=None, pose_type=None):
            success = True
            original_base = robot.base  # Save original base

            if target_pose is not None and base_y is not None:
                # Set base to desired y-position
                robot.base = SE3(0, base_y, 0)
                # Compute inverse kinematics
                ik_result = robot.ikine_LM(target_pose, q0=robot.q, joint_limits=True)
                if not ik_result.success:
                    print(f"Pose at base y={base_y} is unreachable")
                    success = False
                    self.failed_bricks += 1
                robot.base = original_base  # Restore base
            return success

# ---------------- Main ----------------
if __name__ == "__main__":
    env_builder = EnvironmentBuilder()
    controller = Controller(env_builder)
    input("Press enter to start...\n")
    controller.pick_and_place()
    env_builder.env.hold()