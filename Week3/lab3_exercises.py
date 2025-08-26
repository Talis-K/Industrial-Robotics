
# Require libraries
import numpy as np
import matplotlib.pyplot as plt
import time
import keyboard
import swift
from scipy import linalg
from spatialmath.base import plotvol3
from spatialmath import SE3
from spatialgeometry import Cylinder, Cuboid, Sphere
from roboticstoolbox import DHLink, DHRobot, jtraj
from ir_support import line_plane_intersection
from roboticstoolbox.models.DH import Sawyer
from ir_support.robots import DensoVS060

# Useful variables
from math import pi

class Lab3Exercises:
    def __init__(self):
        print("Lab 3 Exercises Starting Point")

    def question1(self):
        # 1.1) Use the picture of the robot to determine the DH parameters
        # Hint: Use the standard DH convention. Remember, all units should be in metres.
        # Use DHLink(d=..., a=..., alpha=..., offset=..., qlim=[lower, upper]) for each link

        link1 = DHLink(d=1, a=0 , alpha=pi/2, qlim=[-pi, pi])
        link2 = DHLink(d=0, a=1, alpha= 0, qlim=[-pi, pi])
        link3 = DHLink(d=0, a=1, alpha=0, qlim=[-pi, pi])

        # 1.2) Generate the robot model using your DH parameters
        # Use: robot = DHRobot([link1, link2, link3], name="Robot1")

        robot = DHRobot([link1, link2, link3], name="Robot1")

        # Then set q = np.zeros(robot.n) or use robot.qz if available
        q = np.zeros(robot.n)

        # Visualise the robot using robot.plot(q, limits=[-2, 2, -2, 2, -2, 2])
        robot.plot(q, limits=[-2, 2, -2, 2, -2, 2])
        plt.close()

        # 1.3) Use the teach window to manually update joint angles and verify visually
        
        fig = robot.teach(q, block=False); 
        while not keyboard.is_pressed('enter'): 
            fig.step(0.05)
        fig.close()

        # 1.4) Capture the joint configuration into a variable q after adjusting in the teach window

        q = robot.q

        # 1.5) Use forward kinematics to calculate the end effector transform: T = robot.fkine(q)
        # print("End effector pose:\n", T)

        T = robot.fkine(q)
        print("End effector pose:\n", T)

        # 1.6) Try one or more of the inverse kinematics solvers in RobotKinematics.py, in this form "result = robot.ikine(T)"
        result = robot.ikine_LM(T)  # or ikine_GN, ikine_NR, ikine_LM etc.
        if result.success:
            q_ik = result.q
            print("1.6) IK solution found:\n", q_ik)
        
            # Verify solution
            T_check = robot.fkine(q_ik)
            print("1.6) Forward kinematics of IK result:\n", T_check)
    
            # Optional: compare position error
            position_error = np.linalg.norm(T.t - T_check.t)
            print(f"1.6) Position error: {position_error:.6f}")
        else:
            print("1.6) IK failed. Reason:", result.reason)


        # 1.7) Get the Jacobian matrix at q using robot.jacob0(q)
        # If using a 3-link robot, consider only the first three rows (translational part)

        J = robot.jacob0(q)
        print("1.7) Jacobian matrix:\n", J)

        # 1.8) Try to invert the Jacobian using linalg.inv(J)

        try:            
            inv_J = linalg.inv(J)
            print("1.8) Inverse of full Jacobian:\n", inv_J)
        except Exception as e:
            print("1.8) Could not invert full Jacobian. Error:", e)

        # 1.8) Try inverting reduced Jacobian (translational part only)
        try:
            J_reduced = J[0:3, 0:3]  # Take only the translational part
            inv_J_reduced = linalg.inv(J_reduced)
            print("1.8) At q:\n", robot.q, "\nReduced Jacobian (translational part): \n", J_reduced, "\n Inverse of reduced Jacobian:\n", inv_J_reduced)
        except Exception as e:
            print("1.8) Could not invert reduced Jacobian. Error:", e)
            
    
       # 1.9) The Jacobian can't be inverted at specific joint configurations (e.g., all joints set to zero). Check to see if there are other configurations where this happens and try pinv.
        q = robot.qz + np.ones(robot.n) * np.deg2rad(90)  # Set all joints to X degrees
        J = robot.jacob0(q)
        try:
            inv_J = linalg.inv(J)            
            print("1.9) At q:\n", robot.q, "\nJacobian: \n", J, "\nInverse of Jacobian:\n", inv_J)
        except Exception as e:
            print("1.9) Error:", e, ". So could not invert full Jacobian at this configuration. Using pseudoinverse instead.")
            inv_J = linalg.pinv(J)
            print("1.9) At q:\n", robot.q, "\nJacobian: \n", J, "\nPseudoinverse of Jacobian:\n", inv_J)

        # 1.10) Visualise how fast the end-effector can move in Cartesian space by plotting the velocity ellipse ellipse showing the reduced 3x3 Jacobian (translational part only)
        try:
            q = robot.qz + np.ones(robot.n) * np.deg2rad(30)
            robot.plot(q, vellipse=True)
            print("1.10) Velocity ellipse plotted")
        except Exception as e:
            print("1.10) Velocity ellipse not plottable:", e)
        input("Press Enter to continue")
        plt.close()

        # 1.11) What do you notice about the shape of the velocity ellipse with different values of q?
        try:
            q = robot.qz + np.ones(robot.n) * np.deg2rad(15)
            print("1.11) Showing the velocity ellipse with different values of q. Press 'space' to stop the velocity ellipse plot")
            for _ in range(30):
                q = q - np.ones(robot.n) * np.deg2rad(1)
                J = robot.jacob0(q)
                if robot.n == 6:
                    inv_J = np.linalg.inv(J)
                    print("q:", robot.q, "\nJ:\n", J, "\nInverse J:\n", inv_J, "\n")
                else:
                    # Use pseudoinverse for robots when other than 6-DOF
                    inv_J = np.linalg.pinv(J) 
                    print("q:", robot.q, "\nJ:\n", J, "\nPseudoinverse J:\n", inv_J, "\n")              

                robot.plot(q, vellipse=True)    
                plt.pause(0.01)
                if keyboard.is_pressed('space'):
                    print("1.11) Stopping velocity ellipse plot")
                    break
        except Exception as e:
            print("1.11) Error:", e)
        
        input("Question finished. Press Enter to continue.\n")        
        plt.close()
        return

    def question2(self):
        # 2.1) Use the Denso VM-6083D-W datasheet to derive the DH parameters

        # 2.2) Add joint limits based on datasheet specifications

        # 2.3) Sample the joint space using 30° intervals — skip joint 6

        # 2.4) Compute fkine for each sample and collect the position of the end effector

        # 2.5) Plot all reachable end-effector positions to visualise the workspace
        # Hint: use ax = plotvol3(); ax.plot(x, y, z, 'r.')

        # (Optional) Reflect on:
        # - Can you speed up the calculation?
        # - Can you remove points below a threshold Z value?
        # - Would a coarser sampling affect accuracy?
        # - Can you use a profiler to identify bottlenecks?
        pass

    def question3(self):
        # 3.1) Load Denso robot model (e.g., DensoVS060) and launch Swift
        # denso_robot = DensoVS060() # Load the Denso VS060 robot model
        # env = swift.Swift()
        # env.launch(realtime=True)
        # env.add(denso_robot)

        # 3.2) Move the robot to q = [0, pi/2, 0, 0, 0, 0]
        # Use fkine to find the end-effector transform (called blast_start_tr)

        # 3.3) Compute end point of mock blast: blast_end = fkine * transl(0, 0, 1)

        # 3.4) Visualise this with a red cylinder aligned along z, starting from end-effector
        # blast_cylinder = Cylinder(radius=0.01, length=1.0, pose=blast_start_tr * SE3(0,0,0.5), color="red")
        # env.add(blast_cylinder)

        # 3.5) Add wall as thin box (e.g., Box at x = 1.5, 0.001m x 4m x 4m size, grey transparent)
        # plane_x = 1.5
        # point_on_plane=[plane_x, 0, 0]
        # # Define bounding box for wall intersection checks
        # plane_bounds = [plane_x - 0.001, plane_x + 0.001, -2, 2, -2, 2]
        # wall = Cuboid(scale=[?,?,?], pose=SE3(point_on_plane), color=[r?,g?,b?,alpha?])
        # env.add(wall)

        # 3.6) Use line_plane_intersection() to find where the blast hits the wall
        # If intersecting, plot the point with a green marker
        # plane_normal=[-1, 0, 0],                
        # intersection_point, check = line_plane_intersection(
        #     plane_normal=?,
        #     point_on_plane=?,
        #     point1_on_line=?,
        #     point2_on_line=?
        # )
        # if check == 1:
        #     intersection_marker = Sphere(radius=0.05, pose=SE3(intersection_point), color="green")
        #     env.add(intersection_marker)

        # 3.7) Loop through 100 random poses with joints 1-3 fixed to [0, pi/2, 0]
        # Generate random values for joints 4-6 and update robot
        # Visualise cylinder and mark intersection points with spheres
        # qlim = denso_robot.qlim.T  # To make the shape (6, 2)                
        # print("Grit-blasting robot running...\nTry press 'e' to terminate.")
        # for _ in range(100):   
        #     if keyboard.is_pressed("e"):
        #         break

        #     # 3.7.1) Ensure that your random joint values stay within the joint limits.
        #     random_component = 0.1 * (np.random.rand(3) - 0.5) * (qlim[3:, 1] - qlim[3:, 0])
        #     goal_q = np.concatenate(([0, np.pi/2, 0], random_component))

            # Check intersection with wall with line_plane_intersection
            # Draw or update the cylinder

        pass

if __name__ == "__main__":
    lab = Lab3Exercises()
    lab.question1()
    # lab.question2()
    # lab.question3()
