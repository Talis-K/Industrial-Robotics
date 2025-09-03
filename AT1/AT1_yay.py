import spatialgeometry as geometry
import numpy as np
import swift
import time
import os
from roboticstoolbox import jtraj, DHLink, DHRobot
from scipy.spatial import ConvexHull
from ir_support import UR3
from spatialmath import SE3, SO3
from spatialgeometry import Cuboid
from math import pi


# ---------------- Gripper Class ----------------
class Gripper:
    def __init__(self, env, robot):
        self.env = env  # Store the Swift environment for rendering
        self.robot = robot  # Store UR3 reference for kinematic calculations
        self.finger_len = 0.05  # Length of each finger
        self.finger_w = 0.01  # Width of each finger
        self.finger_t = 0.04  # Thickness of each finger
        self.max_opening = 0.09  # Maximum opening distance between fingers
        self.closed = 0.07  # Closed distance between fingers
        self.carrying_idx = None  # Index of carried brick, if any

        # Define two prismatic links for the fingers
        finger_L = DHLink(a=0, alpha=0, d=0, theta=0,
                          sigma=1, qlim=[0, self.max_opening/2])  # Left finger prismatic joint
        finger_R = DHLink(a=0, alpha=pi, d=0, theta=0,
                          sigma=1, qlim=[0, self.max_opening/2])  # Right finger prismatic joint

        # Internal DHRobot model for the fingers
        self.model = DHRobot([finger_L, finger_R], name="gripper")  # Create gripper DH model

        # Graphics
        self.finger_L = Cuboid([self.finger_len, self.finger_w, self.finger_t], color=[0.2,0.2,0.2,1])  # Left finger visual
        self.finger_R = Cuboid([self.finger_len, self.finger_w, self.finger_t], color=[0.2,0.2,0.2,1])  # Right finger visual
        self.connector = Cuboid([self.finger_len, self.max_opening - 0.01, self.finger_w], color=[0.2,0.2,0.2,1])  # Connector visual

        env.add(self.finger_L)  # Add left finger to environment
        env.add(self.finger_R)  # Add right finger to environment
        env.add(self.connector)  # Add connector to environment

        # Start closed
        self.q = np.array([self.max_opening/2, -self.max_opening/2])  # Initial joint positions for fingers

    def open(self):
        self.q = np.array([self.max_opening/2, -self.max_opening/2])  # Set fingers to maximum opening
        self.update()  # Update gripper position

    def close(self):
        self.q = np.array([self.closed/2, -self.closed/2])  # Set fingers to closed position
        self.update()  # Update gripper position

    def update(self):
        # Get UR3 tool pose with 90-degree rotation around Z
        T_base = self.robot.fkine(self.robot.q) * SE3.Rz(pi/2) 

        # Forward kinematics for each finger relative to tool frame
        T_L = T_base * self.model.fkine([0, self.finger_t/2]) * SE3(0, self.q[0], 0)  # Left finger pose
        T_R = T_base * self.model.fkine([0, self.finger_t/2]) * SE3(0, self.q[1], 0)  # Right finger pose

        # Update meshes
        self.finger_L.T = T_L  # Set left finger transformation
        self.finger_R.T = T_R  # Set right finger transformation
        self.connector.T = T_base  # Set connector transformation

    def update_with_payload(self, bricks):
        self.update()  # Update gripper position
        if self.carrying_idx is not None:
            T_ee = self.robot.fkine(self.robot.q)  # End-effector pose
            T_offset = SE3.Rx(pi)  # 180-degree rotation around X
            # Adjusted brick pose to center it between gripper fingers
            brick_pose = T_ee * T_offset * SE3(0, 0, -self.finger_w - self.finger_t) 
            bricks[self.carrying_idx].T = brick_pose  # Update brick position


# ---------------- Environment Builder Class ----------------
class EnvironmentBuilder:
    def __init__(self):
        self.y_max = 0.8  # Maximum Y position for the rail carriage
        self.env = swift.Swift()  # Initialize Swift simulation environment
        self.env.launch(realTime=True)  # Launch environment with real-time rendering
        self.env.set_camera_pose([1.5, 1.3, 1.4], [0, 0, -pi/4])  # Set camera view

        # Add environment objects
        self.ground_height = 0.005  # Height of the ground plane
        self.add_fences_and_ground()  # Add fences and ground
        self.add_rail()  # Add rail system
        self.safety = self.load_safety()  # Load safety objects

        # Add robot
        self.robot = UR3()  # Initialize UR3 robot
        self.robot.q = np.array([pi/2, -pi/2, 0, -pi/2, 0, -pi/2])  # Initial joint angles
        # Print initial end-effector pose
        print(f"Starting end effector pose:\n{self.robot.fkine(self.robot.q)}")
        self.robot.base = SE3(0, 0, 0)  # Set base position at origin
        self.robot.links[1].qlim = np.deg2rad([-160, -20])  # Set joint 1 limits
        self.robot.links[2].qlim = np.deg2rad([-180, 180])  # Set joint 2 limits
        # self.robot.links[3].qlim = np.deg2rad([0, 90])  # Commented out joint 3 limit
        for i in range(self.robot.n):
            print(f"Joint {i} limits: lower = {np.rad2deg(self.robot.qlim[0, i]):.2f}°, upper = {np.rad2deg(self.robot.qlim[1, i]):.2f}°")  # Print joint limits
        self.robot.add_to_env(self.env)  # Add robot to environment

        # Add gripper
        self.gripper = Gripper(self.env, self.robot)  # Initialize gripper
        # Update gripper to set initial position
        self.gripper.update()

        # Add bricks
        self.bricks = self.load_bricks()  # Load brick objects

        # Refresh environment to show initial state
        self.env.step(0)

    def add_fences_and_ground(self):
        self.env.add(Cuboid(scale=[3, 0.05, 0.8], pose=SE3(0, 1.5, 0.4 + self.ground_height), color=[0.5, 0.9, 0.5, 0.5]))  # Front fence
        self.env.add(Cuboid(scale=[3, 0.05, 0.8], pose=SE3(0, -1.5, 0.4 + self.ground_height), color=[0.5, 0.9, 0.5, 0.5]))  # Back fence
        self.env.add(Cuboid(scale=[0.05, 3, 0.8], pose=SE3(1.5, 0, 0.4 + self.ground_height), color=[0.5, 0.9, 0.5, 0.5]))  # Right fence
        self.env.add(Cuboid(scale=[0.05, 3, 0.8], pose=SE3(-1.5, 0, 0.4 + self.ground_height), color=[0.5, 0.9, 0.5, 0.5]))  # Left fence
        self.env.add(Cuboid(scale=[3, 3, 2*self.ground_height], pose=SE3(0, 0, 0), color=[0.9, 0.9, 0.5, 1]))  # Ground plane
        self.env.add(Cuboid(scale=[0.9, 0.04, 0.6], pose=SE3(-1, -1.1, 0.3), color=[0.5, 0.5, 0.9, 0.5]))  # Additional object

    def add_rail(self):
        self.env.add(Cuboid(scale=[0.05, 2 * self.y_max, 0.05], pose=SE3(0.1, 0, 0.025 + self.ground_height), color=[0.3, 0.3, 0.35, 1]))  # Left rail
        self.env.add(Cuboid(scale=[0.05, 2 * self.y_max, 0.05], pose=SE3(-0.1, 0, 0.025 + self.ground_height), color=[0.3, 0.3, 0.35, 1]))  # Right rail
        self.rail_carriage = Cuboid(scale=[0.15, 0.15, 0.05], pose=SE3(0.0, 0.0, 0.025), color=[1, 0.4, 0.7, 1])  # Rail carriage
        self.env.add(self.rail_carriage)  # Add carriage to environment

    def load_safety(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))  # Get current directory
        stl_files = ["button.stl", "Fire_extinguisher.stl", "generic_caution.STL"]  # Safety object files
        safety_positions = [
            SE3(-1.3, -1.35, 0.0 + self.ground_height) * SE3.Rx(pi/2), SE3(-1, -1.4, 0.0), SE3(-1.15, -1.48, 0.5) * SE3.Rx(pi/2) * SE3.Ry(pi)
        ]  # Positions with rotations
        safety_colour = [(0.6, 0.0, 0.0, 1.0), (0.5, 0.0, 0.0, 1.0), (1.0, 1.0, 0.0, 1.0)]  # Colors
        safety = []
        for stl_file, pose, colour in zip(stl_files, safety_positions, safety_colour):
            stl_path = os.path.join(current_dir, stl_file)  # Construct file path
            if not os.path.exists(stl_path):
                raise FileNotFoundError(f"STL file not found: {stl_path}")  # Error if file missing
            safety_obj = geometry.Mesh(stl_path, pose=pose * SE3(0, 0, self.ground_height), scale=(0.001, 0.001, 0.001), color=colour)  # Load mesh
            self.env.add(safety_obj)  # Add to environment
            safety.append(safety_obj)  # Store object
        return safety

    def load_bricks(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))  # Get current directory
        stl_path = os.path.join(current_dir, "Brick.stl")  # Brick STL file path
        if not os.path.exists(stl_path):
            raise FileNotFoundError(f"STL file not found: {stl_path}")  # Error if file missing
        brick_positions = [
            SE3(-0.7, -1.2, 0.0), SE3(-0.2, 1.2, 0.0), SE3(-0.2, 0.0, 0.0),
            SE3(-0.2, 0.2, 0.0), SE3(-0.2, -0.2, 0.0), SE3(-0.3, 0.0, 0.0),
            SE3(-0.3, 0.2, 0.0), SE3(-0.3, -0.2, 0.0), SE3(-0.4, 0.0, 0.0),
            SE3(-0.4, 0.2, 0.0), SE3(-0.4, -0.2, 0.0)
        ]  # Brick positions
        bricks = []
        for pose in brick_positions:
            brick = geometry.Mesh(stl_path, pose=pose * SE3(0, 0, self.ground_height), color=(0.4, 0, 0, 1))  # Load brick mesh
            self.env.add(brick)  # Add to environment
            bricks.append(brick)  # Store brick
        return bricks


# ---------------- Controller Class ----------------
class Controller:
    def __init__(self, env_builder: EnvironmentBuilder):
        self.env_builder = env_builder  # Store environment builder reference
        self.env = env_builder.env  # Store environment
        self.robot = env_builder.robot  # Store robot
        self.bricks = env_builder.bricks  # Store bricks
        self.rail_carriage = env_builder.rail_carriage  # Store rail carriage
        self.gripper = env_builder.gripper  # Store gripper
        self.safety = env_builder.safety  # Store safety objects

        self.failed_bricks = 0  # Count of unreachable bricks
        # Wall target poses for placing bricks
        self.wall_pose = [
            SE3(0.3, 0.133, 0.0), SE3(0.3, 0.0, 0.0), SE3(0.3, -0.133, 0.0),
            SE3(0.3, 0.133, 0.033), SE3(0.3, 0.0, 0.033), SE3(0.3, -0.133, 0.033),
            SE3(0.3, 0.133, 0.066), SE3(0.3, 0.0, 0.066), SE3(0.3, -0.133, 0.066)
        ]

    def move_carriage_to_y(self, target_y, steps=25):
        start_y = self.robot.base.t[1]  # Current Y position of base
        for s in np.linspace(0, 1, steps):  # Linear interpolation over steps
            y = (1 - s) * start_y + s * target_y  # Interpolate Y position
            y = np.clip(y, -self.env_builder.y_max, self.env_builder.y_max)  # Clip to rail limits
            self.gripper.update_with_payload(self.bricks)  # Update gripper with payload
            self.robot.base = SE3(0, y, 0)  # Move robot base
            self.rail_carriage.T = SE3(0, y, 0.025)  # Move rail carriage
            self.env.step(0.02)  # Update environment
            time.sleep(0.03)  # Pause for real-time effect

    def pick_and_place(self):
        for i in range(len(self.bricks)):  # Iterate over all bricks
            # Skip if no corresponding wall pose
            if i - self.failed_bricks >= len(self.wall_pose):
                print(f"No wall pose for brick {i+1}, skipping")
                continue

            # Get brick and wall poses
            brick = self.bricks[i]
            brick_pose = SE3(brick.T)  # Current brick position
            wall_pose = self.wall_pose[i - self.failed_bricks]  # Target wall position
            print(f"Processing brick {i+1}: brick_pose={brick_pose.t}, wall_pose={wall_pose.t}")

            # Clamp target y to rail limits
            brick_y = np.clip(brick_pose.t[1], -self.env_builder.y_max, self.env_builder.y_max)
            wall_y = np.clip(wall_pose.t[1], -self.env_builder.y_max, self.env_builder.y_max)

            # Hover over brick
            additional_z = self.gripper.finger_len  # Z offset for hover
            T_pick_hover = brick_pose * SE3(0, 0, 0.1 + additional_z) * SE3.Ry(pi)  # Hover pose
            success, traj = self.check_and_calculate_joint_angles(self.robot, target_pose=T_pick_hover, base_y=brick_y, pose_type="brick hover pose")
            if not success:
                continue

            print(f"Moving base to brick {i+1} y or y_max: {brick_y}")
            self.move_carriage_to_y(brick_y)

            print(f"Moving end-effector to brick {i+1} hover: \n{T_pick_hover}")
            for q in traj:  # Execute trajectory
                self.gripper.update()
                self.robot.q = q
                self.env.step(0.02)
                time.sleep(0.03)

            # Move down to brick
            T_pick = brick_pose * SE3(0, 0, additional_z) * SE3.Ry(pi)  # Pick pose
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
            self.gripper.close()  # Close gripper to grab brick
            self.gripper.carrying_idx = i  # Set carried brick index

            success, traj = self.check_and_calculate_joint_angles(self.robot, target_pose=T_pick_hover, base_y=brick_y, pose_type="brick hover pose")
            if not success:
                continue

            print(f"Moving end-effector to brick {i+1} hover: \n{T_pick_hover}")
            for q in traj:
                self.gripper.update_with_payload(self.bricks)
                self.robot.q = q
                self.env.step(0.02)
                time.sleep(0.03)

            T_place_hover = wall_pose * SE3(0, 0, 0.1 + additional_z) * SE3.Ry(pi)  # Hover pose for placing
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

            T_place = wall_pose * SE3(0, 0, additional_z) * SE3.Ry(pi)  # Place pose
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
            self.gripper.open()  # Open gripper to release brick
            self.gripper.carrying_idx = None  # Clear carried brick index

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
        original_base = robot.base  # Save original base position
        original_q = robot.q.copy()  # Save original joint angles

        if target_pose is not None and base_y is not None:
            # Set base to desired y-position
            robot.base = SE3(0, base_y, 0)
            # Compute IK for the end-effector with joint limits
            ik_result = robot.ikine_LM(target_pose, q0=robot.q, joint_limits=True)
            if not ik_result.success:
                self.failed_bricks += 1
                print(f"Cannot reach {pose_type} at {target_pose.t} from base y={base_y}")
                success = False
                traj_result = []
            else:
                traj_result = jtraj(robot.q, ik_result.q, 30).q  # Generate trajectory
            robot.base = original_base  # Restore base
            robot.q = original_q  # Restore joint angles
            return success, traj_result

    def compute_reach_and_volume(self, env_builder):
        y_max = self.env_builder.y_max  # Get maximum Y range of the rail

        max_x_y_q = np.array([pi/2, 0, 0, -pi/2, 0, -pi/2])  # Joint configuration for max X-Y reach
        max_x_z_q = np.array([pi/2, -pi/2, 0, -pi/2, 0, -pi/2])  # Joint configuration for max X-Z reach
        max_y_z_q = np.array([pi/2, 0, 0, -pi/2, pi/2, -pi/2])  # Joint configuration for max Y-Z reach

        max_distances = {}  # Dictionary to store maximum distances
        for y in [-y_max, y_max]:  # Iterate over rail's Y range
            self.robot.base = SE3(0, y, 0)  # Set base position along rail
            
            # X-Y plane
            T_max_x_y = self.robot.fkine(max_x_y_q)  # Compute forward kinematics for X-Y
            pos_xy = T_max_x_y.t  # Get translation vector
            dist_xy = np.sqrt(pos_xy[0]**2 + pos_xy[1]**2)  # Calculate 2D distance in X-Y plane
            max_distances['x_y'] = max(max_distances.get('x_y', 0), dist_xy)  # Update max distance

            # X-Z plane
            T_max_x_z = self.robot.fkine(max_x_z_q)  # Compute forward kinematics for X-Z
            pos_xz = T_max_x_z.t  # Get translation vector
            dist_xz = np.sqrt(pos_xz[0]**2 + pos_xz[2]**2)  # Calculate 2D distance in X-Z plane
            max_distances['x_z'] = max(max_distances.get('x_z', 0), dist_xz)  # Update max distance

            # Y-Z plane
            T_max_y_z = self.robot.fkine(max_y_z_q)  # Compute forward kinematics for Y-Z
            pos_yz = T_max_y_z.t  # Get translation vector
            dist_yz = np.sqrt(pos_yz[1]**2 + pos_yz[2]**2)  # Calculate 2D distance in Y-Z plane
            max_distances['y_z'] = max(max_distances.get('y_z', 0), dist_yz)  # Update max distance

        print(f"Max XY reach radius: {max_distances['x_y']:.3f} m")  # Print X-Y reach
        print(f"Max XZ reach radius: {max_distances['x_z']:.3f} m")  # Print X-Z reach
        print(f"Max YZ reach radius: {max_distances['y_z']:.3f} m")  # Print Y-Z reach

        # ---- Ellipsoid approximation ----
        r_x_z = max_distances['x_z']  # Use X-Z reach as radius for ellipsoid approximation
        volume = (r_x_z**2) * pi/2 * 2 * y_max  # Approximate volume as ellipsoid: (4/3)πr³ simplified
        print(f"Approximate workspace volume (ellipsoid): {volume:.5f} m³")  # Print approximated volume

        self.robot.base = SE3(0, 0, 0)  # Restore base to origin


# ---------------- Main ----------------
if __name__ == "__main__":
    env_builder = EnvironmentBuilder()  # Create environment builder
    controller = Controller(env_builder)  # Create controller
    controller.compute_reach_and_volume(env_builder)  # Compute reach and volume
    input("Press enter to start...\n")  # Wait for user input
    controller.pick_and_place()  # Execute pick and place operation
    env_builder.env.hold()  # Hold environment open