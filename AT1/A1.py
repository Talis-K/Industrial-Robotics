import spatialgeometry as geometry
import roboticstoolbox as rtb
import numpy as np
import swift
import time
import os
from ir_support import UR3
from spatialmath.base import transl
from spatialmath import SE3
from spatialgeometry import Cuboid, Mesh
from math import pi
    
class LabAT1():
    def do_it(self):
        
        steps = 50                                                                                                             # specify no. of steps
        # specify poses of bricks
        brick_pose = np.array([SE3(-0.2, 0, 0),                                                                                 # pose of brick 1
                               SE3(-0.2, 0.2, 0),                                                                               # pose of brick 2
                               SE3(-0.2, -0.2, 0),                                                                              # pose of brick 3
                               SE3(-0.3, 0, 0),                                                                                 # pose of brick 4  
                               SE3(-0.3, 0.2, 0),                                                                               # pose of brick 5
                               SE3(-0.3, -0.2, 0),                                                                              # pose of brick 6
                               SE3(-0.4, 0, 0),                                                                                 # pose of brick 7    
                               SE3(-0.4, 0.2, 0),                                                                               # pose of brick 8      
                               SE3(-0.4, -0.2, 0)])                                                                             # pose of brick 9    
        # specify poses of wall bricks                      
        wall_pose = np.array([SE3(0.3, 0.15, 0),                                                                                # pose of wall brick 1
                                SE3(0.3, 0, 0),                                                                                 # pose of wall brick 2              
                                SE3(0.3, -0.15, 0),                                                                             # pose of wall brick 3
                                SE3(0.3, 0.15, 0.05),                                                                           # pose of wall brick 4
                                SE3(0.3, 0, 0.05),                                                                              # pose of wall brick 5
                                SE3(0.3, -0.15, 0.05),                                                                          # pose of wall brick 6
                                SE3(0.3, 0.15, 0.1),                                                                            # pose of wall brick 7
                                SE3(0.3, 0, 0.1),                                                                               # pose of wall brick 8
                                SE3(0.3, -0.15, 0.1)])                                                                          # pose of wall brick 9

        #Create a rails
        rail1 = Cuboid(scale=[0.03, 1, 0.02], pose=SE3(0.05, -0.5, 0.015), color=[0.5, 0.5, 0.5, 1], collision=True)  
        rail2 = Cuboid(scale=[0.03, 1, 0.02], pose=SE3(-0.05, -0.5, 0.015), color=[0.5, 0.5, 0.5, 1], collision=True)
        #Create a fence
        fence1 = Cuboid(scale=[3, 0.05, 0.8], pose=SE3(0, 1.5, 0.4), color=[0.5, 0.9, 0.5, 0.5], collision=True)
        fence2 = Cuboid(scale=[3, 0.05, 0.8], pose=SE3(0, -1.5, 0.4), color=[0.5, 0.9, 0.5, 0.5], collision=True)
        fence3 = Cuboid(scale=[0.05, 3, 0.8], pose=SE3(1.5, 0, 0.4), color=[0.5, 0.9, 0.5, 0.5], collision=True)
        fence4 = Cuboid(scale=[0.05, 3, 0.8], pose=SE3(-1.5, 0, 0.4), color=[0.5, 0.9, 0.5, 0.5], collision=True)
        #Create a ground
        ground = Cuboid(scale=[3, 3, 0.01], pose=SE3(), color=[0.8, 0.8, 0.5, 1], collision=True)
        # create an UR3
        robot = UR3()
        robot.q = np.zeros(6) 
        tool_offset = 0.001   # <--- tune this to match actual gripper length
        robot.tool = SE3(0, 0, tool_offset)
        #Import bricks
        current_dir = os.path.dirname(os.path.abspath(__file__))
        stl_path = os.path.join(current_dir, "Brick.stl")

        # create and launch a swift environment
        env = swift.Swift()
        env.launch(realTime=True)
        env.set_camera_pose([1.5, 1.5, 1.5], [0, 0, -pi/4])

        #Add objects and robot to the environment
        robot.add_to_env(env)
        env.add(fence1)
        env.add(fence2)
        env.add(fence3)
        env.add(fence4)
        env.add(ground)
        env.add(rail1)
        env.add(rail2)

        #Import and add bricks
        bricks = []
        brick_offset = 0.025
        for i in range(len(brick_pose)):
            brick = geometry.Mesh(stl_path, pose = brick_pose[i], color = (0.4,0,0,1))
            env.add(brick)
            bricks.append(brick)

        input('Press enter to continue.\n')

        for i in range(len(brick_pose)):                                                                                    # for each brick
            # end effector transform
            T1 = robot.fkine(robot.q)                                                                                       # pose of end effector
            q_robot = robot.ikine_LM(T1, q0 = robot.q).q                                                                    # find joint angles from inverse kinematics
            print(f"Start point: \n{T1}")                                                                                   # print starting pose of end effector

            q_brick = robot.ikine_LM(brick_pose[i] * SE3(0, 0, brick_offset), q0 = robot.q).q                                                         # find joint angles from inverse kinematics
            print (f"Brick: \n{brick_pose[i]}")                                                                             # print pose of brick

            # confirm joint angles are within limits and generate trajectory
            q2_joint_limits = self.check_joint_limits(q_brick, robot)                                                       # check if joint angles are within limits
            q_matrix = rtb.jtraj(q_robot, q_brick, steps).q                                                                 # generate a smooth joint trajectory

            # animate
            for q in q_matrix:                                                                                              # for each set of joint angles in the trajectory
                if q2_joint_limits==True:                                                                                   # if the joint angles are valid       
                    robot.q = q                                                                                             # update robot joint angles                                                                                                                    
                    env.step(0.05)                                                                                          # step the environment
                    time.sleep(0.02)                                                                                        # wait a short time to make the animation visible   
                else:
                    print("ERROR: Goal joint angles exceed limits - cannot move to goal")                                   #print error message
                    break                                                                                                                 
            
            print(f"End point: \n{robot.fkine(robot.q)}")                                                                   # print end pose of end effector         
            print("------------------------------------------------------------------------------------------------")       # print a separator line
            
            # end effector transform
            T1 = robot.fkine(robot.q)                                                                                       # pose of end effector
            q_robot2 = robot.ikine_LM(T1, q0 = robot.q).q 
            print(f"Start point: \n{T1}")                                                                                   # print starting pose of end effector

            # wall transform
            q_wall = robot.ikine_LM(wall_pose[i], q0 = np.zeros(6)).q                                                       # find joint angles from inverse kinematics                    
            print (f"Wall: \n{wall_pose[i]}")                                                                               # print pose of wall    

            # confirm joint angles are within limits abnd generate trajectory
            q_goal_joint_limits = self.check_joint_limits(q_wall, robot)                                                    # check if joint angles are within limits    
            q_matrix = rtb.jtraj(q_robot2, q_wall, steps).q                                                                 # generate a smooth joint trajectory  

            # animate
            for q in q_matrix:                                                                                              # for each set of joint angles in the trajectory
                if q_goal_joint_limits==True:                                                                               # if the joint angles are valid 
                    robot.q = q                                                                                             # update robot joint angles  
                    bricks[i].T = robot.fkine(robot.q)                                                                      # update brick position
                    env.step(0.05)                                                                                          # step the environment
                    time.sleep(0.02)                                                                                        # wait a short time to make the animation visible
                else:
                    print("Goal joint angles exceed limits - cannot move to goal")                                          # print error message
                    break

            #print end robot pose
            print(f"End point: \n{robot.fkine(robot.q)}")                                                                   # print end pose of end effector  
            print("------------------------------------------------------------------------------------------------")       # print a separator line

        env.hold()


    def check_joint_limits(self, Q, robot):
            """
            Helper function: check whether each row of Q (joint configs) is within joint limits.

            Parameters:
            - Q: np.ndarray of shape (N, n)
            - robot: robot object with .qlim attribute of shape (2, n)

            Prints info about any rows that are invalid.
            """
            success = True
            Q = np.atleast_2d(Q)
            qlim = robot.qlim  # shape (2, n), where row 0 = lower, row 1 = upper

            for i, q in enumerate(Q):
                lower_violation = q < qlim[0]
                upper_violation = q > qlim[1]

                if np.any(lower_violation) or np.any(upper_violation):
                    print(f"q[{i}] is out of joint limits:")
                    for j in range(len(q)):
                        if lower_violation[j]:
                            print(f"  Joint {j}: {q[j]:.3f} < lower limit {qlim[0, j]:.3f}")
                        elif upper_violation[j]:
                            print(f"  Joint {j}: {q[j]:.3f} > upper limit {qlim[1, j]:.3f}")
                    success = False
                    
            return success

if __name__ == "__main__":
    LabAT1().do_it()