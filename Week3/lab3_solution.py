# Require libraries
# import math
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

# ---------------------------------------------------------------------------------------#
class Lab3Solution:
    def __init__(self):
        print("Welcome to the Lab 3 Solution!")

    def question1(self):
        plt.close("all")

        def get_model(model):
            if model == 0:  # 3-Link Planar
                return DHRobot([
                    DHLink(d=0, a=1, alpha=0, qlim=[-pi, pi]),
                    DHLink(d=0, a=1, alpha=0, qlim=[-pi, pi]),
                    DHLink(d=0, a=1, alpha=0, qlim=[-pi, pi])
                ], name="3-Link Planar")
            elif model == 1:  # 3-Link 3D
                return DHRobot([
                    DHLink(d=1, a=0, alpha=pi/2, qlim=[-pi/2, pi/2]),
                    DHLink(d=0, a=1, alpha=0, qlim=[-pi/2, pi/2]),
                    DHLink(d=0, a=1, alpha=-pi/2, qlim=[-pi/2, pi/2])
                ], name="3-Link 3D")
            elif model == 2:  # UR10 (AANBOT)
                return DHRobot([
                    DHLink(d=0.1273, a=0, alpha=pi/2),
                    DHLink(d=0, a=-0.612, alpha=0),
                    DHLink(d=0, a=-0.5723, alpha=0),
                    DHLink(d=0.163941, a=0, alpha=pi/2),
                    DHLink(d=0.1157, a=0, alpha=-pi/2),
                    DHLink(d=0.0922, a=0, alpha=0)
                ], name="UR10")
            elif model == 3:  # SPIR
                return DHRobot([
                    DHLink(d=0.09625, a=0, alpha=pi/2, qlim=np.radians([-90, 90])),
                    DHLink(d=0, a=0.27813, alpha=0, offset=1.2981, qlim=np.radians([-74.3575, 105.6425])),
                    DHLink(d=0, a=0, alpha=-pi/2, offset=-2.8689, qlim=np.radians([-90, 90])),
                    DHLink(d=0.23601, a=0, alpha=pi/2, qlim=np.radians([-135, 135])),
                    DHLink(d=0, a=0, alpha=-pi/2, qlim=np.radians([-90, 90])),
                    DHLink(d=0.13435, a=0, alpha=0, qlim=np.radians([-135, 135]))
                ], name="SPIR")
            elif model == 4:
                return Sawyer()

        ## Derive the DH parameters for each of the manipulators provided. Use these to generate a model of the manipulator using the Robot Toolbox. 
        # 0 = Week 2 - 3-Link Planar Robot
        # 1 = 3-Link 3D Robot
        # 2 = AANBOT UR10 arm
        # 3 = SPIR igus arm
        # 4 = Sawyer Robot (7DOF)
        model = 4  # Change this to select robot

        # 1.2) Generate robot model and visualise in 3D
        robot = get_model(model)
        robot.qz = np.zeros(robot.n)  # Set a parameter for the zero joint configuration
        q = robot.qz
        robot.plot(q)

        # 1.3) Use teach interface to explore joint configurations
        input("Press Enter to open teach window...")
        plt.close()
        fig = robot.teach(q, block=False)
        while not keyboard.is_pressed('enter'):
            fig.step(0.05)
        fig.close()

        # 1.4) Store joint angles from teach window
        q = robot.q
        print("1.4) Joint angles from teach window:\n", q * 180 / pi)  # Convert to degrees for readability

        # 1.5) Compute end-effector transformation T = fkine(q)
        T = robot.fkine(q)
        print("1.5) End effector transform:\n", T)

        # 1.6) Try one or more of the inverse kinematics solvers in the form "result = robot.ikine(T)"
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

        # 1.7) Compute Jacobian J = jacob0(q)
        J = robot.jacob0(q)
        print("1.7) Jacobian (base frame):\n", J)

        ## Set print options for better readability
        np.set_printoptions(precision=2, suppress=True)

        # 1.8) Try inverting full Jacobian
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

        time.sleep(1)
        input("Press Enter to continue")
        
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
               
    # ---------------------------------------------------------------------------------------#
    def question2(self):        
        # 2.1 Download the PDF of the robot from Canvas

        # 2.2 Determine the D&H parameters based upon the link measurements on the PDF
        # & 2.3 Determine and include the joint limits in your model
        l1 = DHLink(d=0.475, a=0.180, alpha=-pi/2, offset=0, qlim=np.radians([-170, 170]))
        l2 = DHLink(d=0, a=0.385, alpha=0, offset=-pi/2, qlim=np.radians([-90, 135]))
        l3 = DHLink(d=0, a=-0.100, alpha=pi/2, offset=pi/2, qlim=np.radians([-80, 165]))
        l4 = DHLink(d=0.329 + 0.116, a=0, alpha=-pi/2, offset=0, qlim=np.radians([-185, 185]))
        l5 = DHLink(d=0, a=0, alpha=pi/2, offset=0, qlim=np.radians([-120, 120]))
        l6 = DHLink(d=0.09, a=0, alpha=0, offset=0, qlim=np.radians([-360, 360]))

        denso_robot = DHRobot([l1, l2, l3, l4, l5, l6], name= 'Denso VM6083G')
        denso_robot.name = 'Denso VM6083G'
        fig = denso_robot.plot(np.zeros(denso_robot.n))  # Plot the robot in the zero configuration

        # 2.4 Sample the joint angles within the joint limits at 30 degree increments between each of the joint limits
        # & 2.5 Use fkine to determine the point in space for each of these poses, so that you end up with a big list of points
        step_rads = 30 * pi / 180  # Convert degrees to radians
        qlim = np.transpose(denso_robot.qlim) # Transpose to get the 6x2 matrix for each joint as a row
        # Don't need to worry about joint 6-th
        pointcloud_size = int(np.prod(np.floor((qlim[0:5, 1] - qlim[0:5, 0]) / step_rads + 1)))
        pointcloud = np.zeros((pointcloud_size, 3))
        counter = 0
        start_time = time.time()

        print("Start creating point cloud...")
        for q0 in np.arange(qlim[0,0], qlim[0,1] + step_rads, step_rads):
            for q1 in np.arange(qlim[1,0], qlim[1,1] + step_rads, step_rads):
                for q2 in np.arange(qlim[2,0], qlim[2,1] + step_rads, step_rads):
                    for q3 in np.arange(qlim[3,0], qlim[3,1] + step_rads, step_rads):
                        for q4 in np.arange(qlim[4,0], qlim[4,1] + step_rads, step_rads):
                            # Don't need to worry about joint 5 (6-th), just assume it = 0
                            q5 = 0
                            q = [q0, q1, q2, q3, q4, q5]
                            tr = denso_robot.fkine(q).A
                            if counter == pointcloud_size:
                                break
                            pointcloud[counter,:] = tr[0:3,3]

                            counter += 1
                            if np.mod(counter/pointcloud_size * 100, 1) ==0:
                                end_time = time.time()
                                execution_time = end_time - start_time
                                print(f"After {execution_time} seconds, complete", counter/pointcloud_size*100, "% of pose")
                            
        # Create a 3D model showing where the end effector can be over all these samples
        fig.ax.scatter(pointcloud[:,0], pointcloud[:,1], pointcloud[:,2], c='r', s=1, alpha=0.5)  # Scatter plot of the point cloud
        plt.pause(0.05)
        input("Question finished. Press Enter to continue.\n")
        plt.close()   
        return
    
    # ---------------------------------------------------------------------------------------#
    def question3(self):
        # 3.1 Load the Denso robot model from the toolbox.
        denso_robot = DensoVS060() # Load the Denso VS060 robot model
        env = swift.Swift()
        env.launch(realtime=True)
        env.add(denso_robot)

        # 3.2 Move to the joint state and get the end effector transform
        q = [0, pi/2, 0, 0, 0, 0]
        denso_robot.q = q
        env.step(0.1)
        
        # 3.3 Get the end of the stream with TR * transl (blast stream length (i.e. 1m along z) 
        blast_start_tr = denso_robot.fkine(q)
        blast_start_point = blast_start_tr.t
        blast_end_point = (blast_start_tr @ SE3.Trans(0, 0, 1)).t  # 1 meter forward in z-axis

        # 3.4 Add the blasting stream as a thin red cylinder transformed to the end of the stream (1/2 the length of the cylinder along z-axis)
        blast_cylinder = Cylinder(radius=0.01, length=1.0, pose=blast_start_tr * SE3(0,0,0.5), color="red")
        env.add(blast_cylinder)

        # 3.5 Add a semi-transparent grey wall using a thin Box at x = 1.5
        plane_x = 1.5
        point_on_plane=[plane_x, 0, 0]
        # Define bounding box for wall intersection checks
        plane_bounds = [plane_x - 0.001, plane_x + 0.001, -2, 2, -2, 2]
        wall = Cuboid(scale=[0.001, 4, 4], pose=SE3(point_on_plane), color=[0.6, 0.6, 0.6, 0.4])
        env.add(wall)

        # 3.6 Use provided function to compute intersection with the plane
        plane_normal=[-1, 0, 0],
                
        intersection_point, check = line_plane_intersection(
            plane_normal=plane_normal,
            point_on_plane=point_on_plane,
            point1_on_line=blast_start_point,
            point2_on_line=blast_end_point
        )
        
        if check == 1:
            intersection_marker = Sphere(radius=0.05, pose=SE3(intersection_point), color="green")
            env.add(intersection_marker)

        # 3.7) Repeat the above process while randomly varying the wrist joints 4-to-6 with joints 1-to-3 fixed at [0, π/2, 0] so that the robot consistently faces the wall. 
        qlim = denso_robot.qlim.T  # To make the shape (6, 2)                
        print("Grit-blasting robot running...\nTry press 'e' to terminate.")
        for _ in range(100):   
            if keyboard.is_pressed("e"):
                break

            # 3.7.1) Ensure that your random joint values stay within the joint limits.
            random_component = 0.1 * (np.random.rand(3) - 0.5) * (qlim[3:, 1] - qlim[3:, 0])
            goal_q = np.concatenate(([0, np.pi/2, 0], random_component))

            # 3.7.2) As an extension, you may choose to use jtraj to move to each random goal state smoothly
            joint_trajectory = jtraj(denso_robot.q, goal_q, 10).q

            for q in joint_trajectory:
                denso_robot.q = q
                blast_start_tr = denso_robot.fkine(q)
                blast_start_point = blast_start_tr.t
                blast_end_point = (blast_start_tr @ SE3.Trans(0, 0, 1)).t

                # Check intersection with wall
                intersection_point, check = line_plane_intersection(
                    plane_normal, point_on_plane, blast_start_point, blast_end_point
                )

                if check == 1:
                    # Bound check
                    min_bounds = np.array([plane_bounds[i] for i in [0, 2, 4]])
                    max_bounds = np.array([plane_bounds[i] for i in [1, 3, 5]])
                    in_bounds = np.all(min_bounds < intersection_point) and np.all(intersection_point < max_bounds)

                    if in_bounds:
                        blast_end_point = intersection_point

                        intersection_marker = Sphere(radius=0.05, pose=SE3(intersection_point), color="green")
                        env.add(intersection_marker)

                # # Draw or update the cylinder
                if blast_cylinder:
                    blast_cylinder.T = blast_start_tr * SE3(0,0,0.5)

                env.step(0.02)  # 50Hz refresh

        input("Question finished. Press Enter to continue.\n")   
        return     

# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    lab = Lab3Solution()
    lab.question1()
    lab.question2()
    lab.question3()
    print("Lab 3 Solution completed. You can now close the window.")

    keyboard.unhook_all() 
    plt.close("all")
    time.sleep(0.5)       