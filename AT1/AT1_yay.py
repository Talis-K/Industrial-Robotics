import spatialgeometry as geometry
import numpy as np
import swift
import time
import os
from roboticstoolbox import jtraj, DHLink, DHRobot
from ir_support import UR3
from spatialmath import SE3
from spatialgeometry import Cuboid
from math import pi


# ---------------- Gripper Class ----------------
class Gripper:
    def __init__(self, env, robot):
        self.env = env
        self.robot = robot   # store UR3 reference
        self.finger_len = 0.05
        self.finger_w = 0.01
        self.finger_t = 0.04
        self.max_opening = 0.09
        self.closed = 0.07
        self.carrying_idx = None

        # Define two prismatic links for the fingers
        finger_L = DHLink(a=0, alpha=0, d=0, theta=0,
                          sigma=1, qlim=[0, self.max_opening/2])
        finger_R = DHLink(a=0, alpha=pi, d=0, theta=0,
                          sigma=1, qlim=[0, self.max_opening/2])

        # Internal DHRobot model for the fingers
        self.model = DHRobot([finger_L, finger_R], name="gripper")

        # Graphics
        self.finger_L = Cuboid([self.finger_len, self.finger_w, self.finger_t], color=[0.2,0.2,0.2,1])
        self.finger_R = Cuboid([self.finger_len, self.finger_w, self.finger_t], color=[0.2,0.2,0.2,1])
        self.connector = Cuboid([self.finger_len, self.max_opening - 0.01, self.finger_w], color=[0.2,0.2,0.2,1])

        env.add(self.finger_L)
        env.add(self.finger_R)
        env.add(self.connector)

        # Start closed
        self.q = np.array([self.max_opening/2, -self.max_opening/2])

    def open(self):
        self.q = np.array([self.max_opening/2, -self.max_opening/2])
        self.update()

    def close(self):
        self.q = np.array([self.closed/2, -self.closed/2])
        self.update()

    def update(self):
        # Get UR3 tool pose
        T_base = self.robot.fkine(self.robot.q) * SE3.Rz(pi/2) 

        # Forward kinematics for each finger relative to tool frame
        T_L = T_base * self.model.fkine([0, self.finger_t/2]) * SE3(0, self.q[0], 0)
        T_R = T_base * self.model.fkine([0, self.finger_t/2]) * SE3(0, self.q[1], 0)

        # Update meshes
        self.finger_L.T = T_L
        self.finger_R.T = T_R
        self.connector.T = T_base

    def update_with_payload(self, bricks):
        self.update()
        if self.carrying_idx is not None:
            T_ee = self.robot.fkine(self.robot.q)
            T_offset = SE3.Rx(pi)
            # Adjusted brick pose to center it between gripper fingers, accounting for connector offset
            brick_pose = T_ee * T_offset * SE3(0, 0, -self.finger_w - self.finger_t) 
            bricks[self.carrying_idx].T = brick_pose

# ---------------- Environment Builder Class ----------------
class EnvironmentBuilder:
    def __init__(self):
        self.y_max = 0.8  # Maximum Y position for the rail carriage
        self.env = swift.Swift()
        self.env.launch(realTime=True)
        self.env.set_camera_pose([1.5, 1.3, 1.4], [0, 0, -pi/4])

        # Add environment objects
        self.ground_height = 0.005
        self.add_fences_and_ground()
        self.add_rail()
        self.safety = self.load_safety()

        # Add robot
        self.robot = UR3()
        self.robot.q = np.array([pi/2, -pi/2, 0, -pi/2, 0, -pi/2])  # Only UR3 joints
        #print end effector pose
        print(f"Starting end effector pose:\n{self.robot.fkine(self.robot.q)}")
        self.robot.base = SE3(0, 0, 0)
        self.robot.links[1].qlim = np.deg2rad([-160, -20])  # Set joint limits for the second link
        self.robot.links[2].qlim = np.deg2rad([-180, 180])
        # self.robot.links[3].qlim = np.deg2rad([0, 90])
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
        self.env.add(Cuboid(scale=[3, 3, 2*self.ground_height], pose=SE3(0, 0, 0), color=[0.9, 0.9, 0.5, 1]))
        self.env.add(Cuboid(scale=[0.9, 0.04, 0.6], pose=SE3(-1, -1.1, 0.3), color=[0.5, 0.5, 0.9, 0.5]))

    def add_rail(self):
        self.env.add(Cuboid(scale=[0.05, 2 * self.y_max, 0.05], pose=SE3(0.1, 0, 0.025 + self.ground_height), color=[0.3, 0.3, 0.35, 1]))
        self.env.add(Cuboid(scale=[0.05, 2 * self.y_max, 0.05], pose=SE3(-0.1, 0, 0.025 + self.ground_height), color=[0.3, 0.3, 0.35, 1]))
        self.rail_carriage = Cuboid(scale=[0.15, 0.15, 0.05], pose=SE3(0.0, 0.0, 0.025), color=[1, 0.4, 0.7, 1])
        self.env.add(self.rail_carriage)

    def load_safety(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        stl_files = ["button.stl", "Fire_extinguisher.stl", "generic_caution.STL"]
        safety_positions = [
            SE3(-1.3, -1.35, 0.0 + self.ground_height) * SE3.Rx(pi/2), SE3(-1, -1.4, 0.0), SE3(-1.15, -1.48, 0.5) * SE3.Rx(pi/2) * SE3.Ry(pi)
        ]
        safety_colour = [(0.6, 0.0, 0.0, 1.0), (0.5, 0.0, 0.0, 1.0), (1.0, 1.0, 0.0, 1.0)]
        safety = []
        for stl_file, pose, colour in zip(stl_files, safety_positions, safety_colour):
            stl_path = os.path.join(current_dir, stl_file)
            if not os.path.exists(stl_path):
                raise FileNotFoundError(f"STL file not found: {stl_path}")
            safety_obj = geometry.Mesh(stl_path, pose=pose * SE3(0, 0, self.ground_height), scale=(0.001, 0.001, 0.001), color=colour)
            self.env.add(safety_obj)
            safety.append(safety_obj)
        return safety

    def load_bricks(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        stl_path = os.path.join(current_dir, "Brick.stl")
        if not os.path.exists(stl_path):
            raise FileNotFoundError(f"STL file not found: {stl_path}")
        brick_positions = [
            SE3(-0.7, -1.2, 0.0), SE3(-0.2, 1.2, 0.0), SE3(-0.2, 0.0, 0.0),
            SE3(-0.2, 0.2, 0.0), SE3(-0.2, -0.2, 0.0), SE3(-0.3, 0.0, 0.0),
            SE3(-0.3, 0.2, 0.0), SE3(-0.3, -0.2, 0.0), SE3(-0.4, 0.0, 0.0),
            SE3(-0.4, 0.2, 0.0), SE3(-0.4, -0.2, 0.0)
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
            SE3(0.3, 0.133, 0.0), SE3(0.3, 0.0, 0.0), SE3(0.3, -0.133, 0.0),
            SE3(0.3, 0.133, 0.033), SE3(0.3, 0.0, 0.033), SE3(0.3, -0.133, 0.033),
            SE3(0.3, 0.133, 0.066), SE3(0.3, 0.0, 0.066), SE3(0.3, -0.133, 0.066)
        ]

    def move_carriage_to_y(self, target_y, steps=25):
        start_y = self.robot.base.t[1]
        for s in np.linspace(0, 1, steps):
            y = (1 - s) * start_y + s * target_y
            y = np.clip(y, -self.env_builder.y_max, self.env_builder.y_max)
            self.gripper.update_with_payload(self.bricks)
            self.robot.base = SE3(0, y, 0)
            self.rail_carriage.T = SE3(0, y, 0.025)
            self.env.step(0.02)
            time.sleep(0.03)

    def pick_and_place(self):
        for i in range(len(self.bricks)):
            # Skip if no corresponding wall pose
            if i - self.failed_bricks >= len(self.wall_pose):
                print(f"No wall pose for brick {i+1}, skipping")
                continue

            # Get brick and wall poses
            brick = self.bricks[i]
            brick_pose = SE3(brick.T)
            wall_pose = self.wall_pose[i - self.failed_bricks]
            print(f"Processing brick {i+1}: brick_pose={brick_pose.t}, wall_pose={wall_pose.t}")

            # Clamp target y to rail limits
            brick_y = np.clip(brick_pose.t[1], -self.env_builder.y_max, self.env_builder.y_max)
            wall_y = np.clip(wall_pose.t[1], -self.env_builder.y_max, self.env_builder.y_max)

            # Hover over brick, adding offset for additional Z if needed
            additional_z = self.gripper.finger_len  # Adjust Z based on connector offset (since negative offset means lower connector, raise EE)
            T_pick_hover = brick_pose * SE3(0, 0, 0.1 + additional_z) * SE3.Ry(pi)
            success, traj = self.check_and_calculate_joint_angles(self.robot, target_pose=T_pick_hover, base_y=brick_y, pose_type="brick hover pose")
            if not success:
                continue

            print(f"Moving base to brick {i+1} y or y_max: {brick_y}")
            self.move_carriage_to_y(brick_y)

            print(f"Moving end-effector to brick {i+1} hover: \n{T_pick_hover}")
            for q in traj:
                self.gripper.update()
                self.robot.q = q
                self.env.step(0.02)
                time.sleep(0.03)

            # Move down to brick
            T_pick = brick_pose * SE3(0, 0, additional_z) * SE3.Ry(pi)
            success, traj = self.check_and_calculate_joint_angles(self.robot, target_pose=T_pick, base_y=brick_y, pose_type="brick pose")
            if not success:
                continue

            print(f"Moving end-effector to brick {i+1} pose: \n{T_pick}")
            for q in traj:
                self.gripper.update()
                self.robot.q = q
                self.env.step(0.02)
                time.sleep(0.03)

            print("Closing gripper")
            self.gripper.close()
            self.gripper.carrying_idx = i

            success, traj = self.check_and_calculate_joint_angles(self.robot, target_pose=T_pick_hover, base_y=brick_y, pose_type="brick hover pose")
            if not success:
                continue

            print(f"Moving end-effector to brick {i+1} hover: \n{T_pick_hover}")
            for q in traj:
                self.gripper.update_with_payload(self.bricks)
                self.robot.q = q
                self.env.step(0.02)
                time.sleep(0.03)

            T_place_hover = wall_pose * SE3(0, 0, 0.1 + additional_z) * SE3.Ry(pi)
            success, traj = self.check_and_calculate_joint_angles(self.robot, target_pose=T_place_hover, base_y=wall_y, pose_type="wall hover pose")
            if not success:
                continue

            print(f"Moving base to wall {i+1} y or y_max: {wall_y}")
            self.move_carriage_to_y(wall_y)

            print(f"Moving end-effector to wall {i+1} hover: \n{T_place_hover}")
            for q in traj:
                self.gripper.update_with_payload(self.bricks)
                self.robot.q = q
                self.env.step(0.02)
                time.sleep(0.03)

            T_place = wall_pose * SE3(0, 0, additional_z) * SE3.Ry(pi)
            success, traj = self.check_and_calculate_joint_angles(self.robot, target_pose=T_place, base_y=wall_y, pose_type="wall pose")
            if not success:
                continue

            print(f"Moving end-effector to wall {i+1} pose: \n{T_place}")
            for q in traj:
                self.gripper.update_with_payload(self.bricks)
                self.robot.q = q
                self.env.step(0.02)
                time.sleep(0.03)

            print("Opening gripper")
            self.gripper.open()
            self.gripper.carrying_idx = None

            success, traj = self.check_and_calculate_joint_angles(self.robot, target_pose=T_place_hover, base_y=wall_y, pose_type="wall hover pose")
            if not success:
                continue

            print(f"Moving end-effector to wall {i+1} hover: \n{T_place_hover}")
            for q in traj:
                self.gripper.update()
                self.robot.q = q
                self.env.step(0.02)
                time.sleep(0.03)

    def check_and_calculate_joint_angles(self, robot, target_pose=None, base_y=None, pose_type=None):
        success = True
        original_base = robot.base  # Save original base
        original_q = robot.q.copy()  # Save original joint angles

        if target_pose is not None and base_y is not None:
            # Set base to desired y-position
            robot.base = SE3(0, base_y, 0)
            # Compute IK for the end-effector
            ik_result = robot.ikine_LM(target_pose, q0=robot.q, joint_limits=True)
            if not ik_result.success:
                self.failed_bricks += 1
                print(f"Cannot reach {pose_type} at {target_pose.t} from base y={base_y}")
                success = False
                traj_result = []
            else:
                traj_result = jtraj(robot.q, ik_result.q, 30).q
            robot.base = original_base  # Restore base
            robot.q = original_q  # Restore joint angles
            return success, traj_result

    def compute_reach_and_volume(self, env_builder):
        y_max = self.env_builder.y_max

        max_x_y_q = np.array([pi/2, 0, 0, -pi/2, 0, -pi/2])
        max_x_z_q = np.array([pi/2, -pi/2, 0, -pi/2, 0, -pi/2])
        max_y_z_q = np.array([pi/2, 0, 0, -pi/2, pi/2, -pi/2])

        max_distances = {}
        for y in [-y_max, y_max]:
            self.robot.base = SE3(0, y, 0)
            
            # X-Y plane
            T_max_x_y = self.robot.fkine(max_x_y_q)
            pos_xy = T_max_x_y.t
            dist_xy = np.sqrt(pos_xy[0]**2 + pos_xy[1]**2)
            max_distances['x_y'] = max(max_distances.get('x_y', 0), dist_xy)

            # X-Z plane
            T_max_x_z = self.robot.fkine(max_x_z_q)
            pos_xz = T_max_x_z.t
            dist_xz = np.sqrt(pos_xz[0]**2 + pos_xz[2]**2)
            max_distances['x_z'] = max(max_distances.get('x_z', 0), dist_xz)

            # Y-Z plane
            T_max_y_z = self.robot.fkine(max_y_z_q)
            pos_yz = T_max_y_z.t
            dist_yz = np.sqrt(pos_yz[1]**2 + pos_yz[2]**2)
            max_distances['y_z'] = max(max_distances.get('y_z', 0), dist_yz)

        print(f"Max XY reach radius: {max_distances['x_y']:.3f} m")
        print(f"Max XZ reach radius: {max_distances['x_z']:.3f} m")
        print(f"Max YZ reach radius: {max_distances['y_z']:.3f} m")

        # ---- Ellipsoid approximation ----
        r_x_z = max_distances['x_z']      # X reach

        volume = (r_x_z**2) * pi/2 * 2 *y_max
        print(f"Approximate workspace volume (ellipsoid): {volume:.5f} m³")

        self.robot.base = SE3(0, 0, 0)


# ---------------- Main ----------------
if __name__ == "__main__":
    env_builder = EnvironmentBuilder()
    controller = Controller(env_builder)
    controller.compute_reach_and_volume(env_builder)
    input("Press enter to start...\n")
    controller.pick_and_place()
    env_builder.env.hold()