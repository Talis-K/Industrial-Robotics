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
        self.finger_len = 0.08
        self.finger_w = 0.01
        self.finger_t = 0.01
        self.ee_to_finger_tip = 0.07
        self.opening = 0.06
        self.carrying_idx = None

        # Create finger geometry
        self.finger_L = Cuboid(scale=[self.finger_len, self.finger_w, self.finger_t],
                               pose=SE3(), color=[0.2, 0.2, 0.2, 1])
        self.finger_R = Cuboid(scale=[self.finger_len, self.finger_w, self.finger_t],
                               pose=SE3(), color=[0.2, 0.2, 0.2, 1])
        env.add(self.finger_L)
        env.add(self.finger_R)

    def open(self):
        self.opening = 0.06
        self.update()

    def close(self):
        self.opening = 0.0
        self.update()

    def update(self):
        """Update gripper geometry according to robot pose."""
        T_ee = self.robot.fkine(self.robot.q)

        # Rotate gripper 180° about X so fingers point downward
        T_offset = SE3.Rx(pi) * SE3(self.ee_to_finger_tip, 0, -0.02)

        half_gap = self.opening / 2
        T_fL = T_ee * T_offset * SE3(0,  half_gap + 0.5 * self.finger_w, 0)
        T_fR = T_ee * T_offset * SE3(0, -half_gap - 0.5 * self.finger_w, 0)

        self.finger_L.T = T_fL
        self.finger_R.T = T_fR


    def update_with_payload(self, bricks):
        self.update()
        if self.carrying_idx is not None:
            T_ee = self.robot.fkine(self.robot.q)
            T_offset = SE3.Rx(pi) * SE3(self.ee_to_finger_tip, 0, -0.02)
            bricks[self.carrying_idx].T = T_ee * T_offset * SE3(0, 0, -0.015)



# ---------------- Environment Builder Class ----------------
class EnvironmentBuilder:
    def __init__(self):
        self.env = swift.Swift()
        self.env.launch(realTime=True)
        self.env.set_camera_pose([1.5, 1.3, 1.4], [0, 0, -pi/4])

        # Add environment objects
        self.add_fences_and_ground()
        self.add_rail()

        # Add robot
        self.robot = UR3()
        self.robot.q = np.zeros(6)
        self.robot.base = SE3(0, 0, 0)
        UR3.links[1].qlim = np.deg2rad([-90,  90])
        #print q lim of each joint
            # Print joint limits for each joint
        for i in range(self.robot.n):
            print(f"Joint {i} limits: lower = {np.rad2deg(self.robot.qlim[0, i]):.2f}°, upper = {np.rad2deg(self.robot.qlim[1, i]):.2f}°")
        self.robot.add_to_env(self.env)

        # Add gripper
        self.gripper = Gripper(self.env, self.robot)

        # Add bricks
        self.bricks = self.load_bricks()

    def add_fences_and_ground(self):
        self.env.add(Cuboid(scale=[3, 0.05, 0.8], pose=SE3(0,  1.5, 0.4), color=[0.5, 0.9, 0.5, 0.5]))
        self.env.add(Cuboid(scale=[3, 0.05, 0.8], pose=SE3(0, -1.5, 0.4), color=[0.5, 0.9, 0.5, 0.5]))
        self.env.add(Cuboid(scale=[0.05, 3, 0.8], pose=SE3( 1.5, 0, 0.4), color=[0.5, 0.9, 0.5, 0.5]))
        self.env.add(Cuboid(scale=[0.05, 3, 0.8], pose=SE3(-1.5, 0, 0.4), color=[0.5, 0.9, 0.5, 0.5]))
        self.env.add(Cuboid(scale=[3, 3, 0.01], pose=SE3(), color=[0.8, 0.8, 0.5, 1]))

    def add_rail(self):
        self.env.add(Cuboid(scale=[0.2, 3.2, 0.05], pose=SE3(0.0, 0.0, 0.025), color=[0.3, 0.3, 0.35, 1]))
        self.rail_carriage = Cuboid(scale=[0.25, 0.25, 0.08], pose=SE3(0.0, 0.0, 0.09), color=[0.4, 0.4, 0.7, 1])
        self.env.add(self.rail_carriage)

    def load_bricks(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        stl_path = os.path.join(current_dir, "Brick.stl")
        brick_positions = [
            SE3(-0.2, 0.0, 0.0), SE3(-0.2, 0.2, 0.0), SE3(-0.2, -0.2, 0.0),
            SE3(-0.3, 0.0, 0.0), SE3(-0.3, 0.2, 0.0), SE3(-0.3, -0.2, 0.0),
            SE3(-0.4, 0.0, 0.0), SE3(-0.4, 0.2, 0.0), SE3(-0.4, -0.2, 0.0)
        ]
        bricks = []
        for pose in brick_positions:
            brick = geometry.Mesh(stl_path, pose=pose, color=(0.4, 0, 0, 1))
            self.env.add(brick)
            bricks.append(brick)
        return bricks


# ---------------- Controller Class ----------------
class Controller:
    def __init__(self, env_builder: EnvironmentBuilder):
        self.env = env_builder.env
        self.robot = env_builder.robot
        self.bricks = env_builder.bricks
        self.rail_carriage = env_builder.rail_carriage
        self.gripper = env_builder.gripper

        # Wall target poses
        self.wall_pose = [
            SE3(0.3, 0.15, 0.00), SE3(0.3, 0.00, 0.00), SE3(0.3, -0.15, 0.00),
            SE3(0.3, 0.15, 0.05), SE3(0.3, 0.00, 0.05), SE3(0.3, -0.15, 0.05),
            SE3(0.3, 0.15, 0.10), SE3(0.3, 0.00, 0.10), SE3(0.3, -0.15, 0.10)
        ]

    def move_carriage_to_y(self, target_y, steps=25):
        start_y = self.robot.base.t[1]
        for s in np.linspace(0, 1, steps):
            y = (1 - s) * start_y + s * target_y
            self.robot.base = SE3(0, y, 0)
            self.rail_carriage.T = SE3(0, y, 0.09)
            self.gripper.update_with_payload(self.bricks)
            self.env.step(0.02)
            time.sleep(0.02)

    def pick_and_place(self):
        for i, brick in enumerate(self.bricks):
            brick_pose = SE3(brick.T)
            wall_pose = self.wall_pose[i]

            # 1) Move base close to brick
            self.move_carriage_to_y(brick_pose.t[1])

            # 2) Approach hover
            T_hover = SE3(brick_pose.t[0], brick_pose.t[1], brick_pose.t[2] + 0.05)
            q_hover = self.robot.ikine_LM(T_hover, q0=self.robot.q).q
            for q in jtraj(self.robot.q, q_hover, 30).q:
                self.robot.q = q
                self.gripper.update()
                self.env.step(0.02)
                time.sleep(0.02)

            #print q for each joint
            print(self.robot.q)

            # 3) Close gripper
            self.gripper.close()
            self.gripper.carrying_idx = i

            # 4) Move base to wall
            self.move_carriage_to_y(wall_pose.t[1])

            # 5) Place brick
            T_place = wall_pose
            q_place = self.robot.ikine_LM(T_place, q0=self.robot.q).q
            for q in jtraj(self.robot.q, q_place, 30).q:
                self.robot.q = q
                self.gripper.update_with_payload(self.bricks)
                self.env.step(0.02)
                time.sleep(0.02)

            # 6) Release
            self.gripper.open()
            self.gripper.carrying_idx = None


# ---------------- Main ----------------
if __name__ == "__main__":
    env_builder = EnvironmentBuilder()
    controller = Controller(env_builder)
    input("Press enter to start...\n")
    controller.pick_and_place()
    env_builder.env.hold()
