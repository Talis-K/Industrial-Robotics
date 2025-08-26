# Require libraries
import numpy as np
import matplotlib.pyplot as plt
import time
import swift
from spatialmath.base import *
from spatialmath import SE3
from roboticstoolbox import models, jtraj, trapezoidal
from spatialgeometry import Sphere, Arrow

# Useful variables
from math import pi

# -----------------------------------------------------------------------------------#
class Lab4Solution():
    def __init__(self):
        print("Lab 4 Solution - Questions 2 and 3")

    
    def question2_and_3(self):
        """
        Lab 4 - Question 2 & 3 - Inverse Kinematics & Joint Interpolation
        """
        ## Options
        interpolation = 2           # 1 = Quintic Polynomial, 2 = Trapezoidal Velocity                       
        steps = 50                  # Specify no. of steps
        point_freq = 1              # Frequency of points to plot end-effector path (1 = every point, 2 = every two points, etc) - increase if CPU lagging 

        # 2.1) (also 3.1) Load a model of the Puma 560 robot
        p560 = models.Puma560()
        qlim = np.transpose(p560.qlim)

        # 2.2-2.5) (also 3.2) Define end-effector poses as a 4x4 Homogeneous Transformation Matrix,
        # and solve the inverse kinematics to get the required joint angles
        # Note that the returned solution may not be within joint limits. In this case, you should try a more accurate q0 (guess)
        # E.g. You can use teach to estimate a joint solution for a cartesian XYZ location
        T1 = transl(0.5,-0.4,0.5)                                                                               # Create translation matrix
        q1 = p560.ikine_LM(T1, q0 = np.zeros([1,6])).q                                                          # Derive joint angles for required end-effector transformation
        q1_in_joint_limits = self.check_joint_limits(q1, p560)
        print(f"q1 within joint limits" if q1_in_joint_limits == True else f"q1 not within joint limits: {q1}")

        T2 = transl(0.5,0.4,0.1)                                                                                # Define a translation matrix                                           
        q2 = p560.ikine_LM(T2, q0 = q1).q                                                                       # Use inverse kinematics to get the joint angles
        q2_in_joint_limits = self.check_joint_limits(q2, p560)
        print(f"q2 within joint limits" if q2_in_joint_limits == True else f"q2 not within joint limits: {q2}")

        input('Press enter to continue to Question 3\n')

        # 3.3) Generate a matrix of interpolated joint angles between q1 and q2 using the
        # Quintic Polynomial and Trapezoidal Velocity methods
        if interpolation == 1:
            q_matrix = jtraj(q1, q2, steps).q
        elif interpolation == 2:
            s = trapezoidal(0, 1, steps).q                                                                      # Create the scalar function
            q_matrix = np.empty((steps, 6))                                                                     # Create memory allocation for variables
            for i in range(steps):
                q_matrix[i, :] = (1 - s[i]) * q1 + s[i] * q2                                                    # Generate interpolated joint angles
        else:
            raise ValueError("interpolation = 1 for Quintic Polynomial, or 2 for Trapezoidal Velocity")

        # 3.4) Animate the generated trajectory and the Puma 560 robot.
        # NEW: Do this in Swift - Launch Swift environment, add robot and plot at all zeros
        env = swift.Swift()
        env.launch(realtime=True)
        p560.q = np.zeros(p560.n)
        env.set_camera_pose([1.25, 1.25, 1.25], [0, 0, 0])  # (position, look-at)
        env.add(p560, readonly=False)
        
        # Create axes (either a bit brighter than default, or with arrows)
        axes = self.create_minimal_axes(env, scale=5.0, radius=0.01, head_scale=0.05, head_radius=0.25)
        # Update environment
        env.step()
        time.sleep(0.05)
        input('Press enter to continue.\n')

        # Now plot the robot in the initial (first) joint state of the calculated trajectory
        p560.q = q1
        env.step(0.05)
        input('Press enter to continue.\n')

        points = []     # Creating a list to hold 'Sphere' objects (acting as points in Swift env)
        # Plot the motion between poses, draw a red line of the end-effector path
        for i, q in enumerate(q_matrix):
            p560.q = q      # Update robot joint state
            env.step(0.05)  # Update environment
    
            # Create sphere (radius = 0.05m) at EE position for every 'point_freq' steps (to trace path)
            if(i % point_freq == 0):
                new_point = Sphere(radius=0.05, color=[1.0, 0.0, 0.0, 1.0])
                new_point.T = p560.fkine(p560.q).A
                env.add(new_point)
                points.append(new_point)

        # Note the end position of the trajectory and compare it to the desired location
        final_ee_tr = p560.fkine(p560.q)
        print(f'End effector pose at end of trajectory: \n{final_ee_tr}')
        print(f'Position error (X, Y, Z) = {np.round(T2[0, 3] - final_ee_tr.A[0, 3], 3)}, {np.round(T2[1, 3] - final_ee_tr.A[1, 3], 3)}, {np.round(T2[2, 3] - final_ee_tr.A[2, 3], 3)}')

        input("Press enter to continue.\n")


        # 3.5) Create matrices of the joint velocities and acceleration
        velocity = np.zeros([steps, 6])
        acceleration = np.zeros([steps, 6])
        for i in range(1,steps):
            velocity[i,:] = q_matrix[i,:] - q_matrix[i-1,:]
            acceleration[i,:] = velocity[i,:] - velocity[i-1,:]

        # 3.6) Plot the joint angles, velocities, and accelerations.
        # Plot joint angles
        plt.figure(1)
        for i in range(6):
            plt.subplot(3, 2, i+1)
            plt.plot(q_matrix[:, i], 'k', linewidth=1)
            plt.title('Joint ' + str(i+1))
            plt.xlabel('Step')
            plt.ylabel('Joint Angle (rad)')
            plt.axhline(qlim[i, 0], color='r')  # Plot lower joint limit
            plt.axhline(qlim[i, 1], color='r')  # Plot upper joint limit

        # Plot joint velocities
        plt.figure(2)
        for i in range(6):
            plt.subplot(3, 2, i+1)
            plt.plot(velocity[:, i], 'k', linewidth=1)
            plt.title('Joint ' + str(i+1))
            plt.xlabel('Step')
            plt.ylabel('Joint Velocity')

        # Plot joint accelerations
        plt.figure(3)
        for i in range(6):
            plt.subplot(3, 2, i+1)
            plt.plot(acceleration[:, i], 'k', linewidth=1)
            plt.title('Joint ' + str(i+1))
            plt.xlabel('Step')
            plt.ylabel('Joint Acceleration')

        plt.show()      # Display all plots

        input('Enter to exit\n')
        env.close()


    def create_minimal_axes(self, env, scale=3.0, radius=0.01, head_scale=0.5, head_radius=0.1):
        """
        Create positive axes with minimal objects (only 3 arrows)
        Blue: Z
        Red: X
        Green: Y
        """
        axes_objects = []
        
        # Positive X-axis (Red)
        x_pos = Arrow(length=scale, radius=radius, head_length=head_scale, head_radius=head_radius, color=[1.0, 0.0, 0.0, 1.0])
        x_pos.T = SE3.Ry(pi/2).A  # Points along +X from origin
        env.add(x_pos)
        axes_objects.append(x_pos)

        # Positive Y-axis (Green)
        y_pos = Arrow(length=scale, radius=radius, head_length=head_scale, head_radius=head_radius, color=[0.0, 1.0, 0.0, 1.0])
        y_pos.T = SE3.Rx(-pi/2).A  # Points along +Y from origin
        env.add(y_pos)
        axes_objects.append(y_pos)

        # Positive Z-axis (Blue)
        z_pos = Arrow(length=scale, radius=radius, head_length=head_scale, head_radius=head_radius, color=[0.0, 0.0, 1.0, 1.0])
        z_pos.T = SE3(0, 0, 0).A  # Points along +Z from origin
        env.add(z_pos)
        axes_objects.append(z_pos)
        
        return axes_objects
    

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

# ---------------------------------------------------------------------------------------#
# Main block
if __name__ == "__main__":
    soln = Lab4Solution()
    soln.question2_and_3()

    print("Lab 4, Questions 2 and 3 Solution completed. You can now close the window.")
    plt.close("all")
    time.sleep(0.5)