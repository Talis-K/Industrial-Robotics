# Require libraries
import numpy as np
import matplotlib.pyplot as plt
import threading
import time
import swift
from spatialmath.base import *
from spatialmath import SE3
from spatialgeometry import Sphere, Arrow, Mesh
from roboticstoolbox import DHLink, DHRobot, models
from ir_support import CylindricalDHRobotPlot
import os

# Useful variables
from math import pi

# -----------------------------------------------------------------------------------#
class Lab4Solution():
    def __init__(self):
        print("Lab 4 Solution - Question 1")
        self.stop_event = threading.Event()         # Event to end teach mode when 'enter' pressed


    def wait_for_enter(self):
        '''
        Helper threaded function to detect keypress without needing keyboard library
        '''
        try:
            #print("Press Enter to stop.\n")
            input()
        except EOFError:
            pass
        self.stop_event.set()

    
    def question1(self):
        """
        Lab 4 - Questions 1: 3-link planar robot - draw a line then circle
        """
        plt.close('all')

        ## Options
        point_freq = 1    # Frequency of points to plot end-effector path (1 = every point, 2 = every two points, etc) - increase if CPU lagging          
    
        # 1.1) Make a 3DOF planar arm model
        # Define the robot using DH parameters
        l1 = DHLink(d=0, a=1, alpha=0, qlim=[-pi, pi])
        l2 = DHLink(d=0, a=1, alpha=0, qlim=[-pi, pi])
        l3 = DHLink(d=0, a=1, alpha=0, qlim=[-pi, pi])
        robot = DHRobot([l1, l2, l3], name='my_robot')

        # Give the robot a cylinder mesh (links) to display in Swift environment
        cyl_viz = CylindricalDHRobotPlot(robot, cylinder_radius=0.05, color="#3478f6")
        robot = cyl_viz.create_cylinders()

        # 1.2) Rotate the base around the X axis so the Z axis faces down ways (and the Y-axis is flipped)
        # Note: You can also rotate about the Y axis so the Z axis faces down, but this would also flip
        # the X axis (so the robot would extend in negative X, not positive X) - this isn't necessarily an issue
        robot.base = trotx(pi)

        # 1.3, 1.4) Set workspace, scale and initial joint state, then plot and teach
        workspace = [-3, 3, -3, 3, -0.05, 2]
        
        q = np.zeros([1,3])

        robot.plot(q, limits= workspace)
        input("Enter to teach and hit Enter again to continue\n")
        plt.close()
        fig = robot.teach(q, limits= workspace, block = False)

        # Continuously update the teach figure while it is being used (every 0.05s)
        input_thread = threading.Thread(target=self.wait_for_enter)
        input_thread.start()
        while not self.stop_event.is_set():
            fig.step(0.05)

        self.stop_event.clear()
        plt.close('all')
        input_thread.join()


        # 1.5, 1.6) Get a solution for the end effector at [-0.75,-0.5,0]. 
        # Note that from teach you can see that there is no way to affect the Z, roll or pitch values. Since the pen is parallel to the Z axis then we
        # don't care about the yaw angle (rotating the pen about the Z-axis does not change the result). Therefore, we can mask out Z, roll, pitch, yaw
        # in the ikine function.
        # NEW: Do this in Swift - Launch Swift environment, add robot
        env = swift.Swift()
        env.launch(realtime=True)
        env.add(robot)

        # Create axes (either a bit brighter than default, or with arrows)
        axes = self.create_minimal_axes(env, scale=5.0, radius=0.01, head_scale=0.05, head_radius=0.25)
        env.set_camera_pose([3, 3, 2], [0, 0, 0])  # (position, look-at)
        input("Press enter to continue (once swift environment has loaded)\n")

        # Get solution of joint angles at EE = [-0.75, -0.5, 0]
        new_q = robot.ikine_LM(transl([-0.75,-0.5,0]), q0=q, mask= [1,1,0,0,0,0]).q
        # Update robot joint angles
        robot.q = new_q
        env.step(0.05)
        # 1.7) Check how close it got to the goal transform of transl(-0.75,-0.5,0)
        print('Fkine solution:\n', robot.fkine(new_q))
        input("Press enter to continue\n")

        # 1.8, 1.9) Go through a loop using the previous joint as the guess to draw a line from [-0.75,-0.5,0] to [-0.75,0.5,0]
        points = []     # Creating a list to hold 'Sphere' objects (acting as points in Swift env)
        for i, y in enumerate(np.arange(-0.5,0.5+0.05, 0.05)):
            # Get joint angles for specified EE position + update robot
            new_q = robot.ikine_LM(transl(-0.75,y,0), q0 = new_q, mask=[1,1,0,0,0,0]).q
            robot.q = new_q 
            env.step(0.05)

            # Create sphere (radius = 0.05m) at EE position for every 'point_freq' steps (to trace path)
            if(i % point_freq == 0):
                new_point = Sphere(radius=0.05, color=[1.0, 0.0, 0.0, 1.0])
                new_point.T = robot.fkine(new_q).A
                env.add(new_point)
                points.append(new_point)

                # Update environment
                env.step()
                time.sleep(0.05)

        input("Press Enter to continue\n")

        # Reset the environment to remove spheres, have to re-add robot
        env.reset()
        points.clear()        # Clear 'points' list
        env.add(robot)
        env.set_camera_pose([2, 2, 2], [0, 0, 0])   # set camera (position, look-at)

        # 1.10) Using ikine to get the newQ and fkine to determine the actual point, 
        # and animate to move the robot to “draw” a circle around the robot with a radius of 0.5m
        r = 0.5
        for i, deg in enumerate(np.arange(0, 360, 5)):
            radians = deg*pi/180            # Convert to radians

            # Calculate x, y locations
            x = r*np.cos(radians)
            y = r*np.sin(radians)

            # Inverse Kinematics to get joint configuration for robot end-effector to be at each location
            new_q = robot.ikine_LM(transl(x,y,0), q0= new_q, mask=[1,1,0,0,0,0]).q
            robot.q = new_q
            env.step(0.05)

            # Plot spheres 
            if(i % point_freq == 0):
                new_point = Sphere(radius=0.05, color=[1.0, 0.0, 0.0, 1.0])
                new_point.T = transl(x, y, 0)
                env.add(new_point)
                points.append(new_point)

                # Update environment
                env.step()
                time.sleep(0.05)

        input("Press Enter to continue\n")
        
        # 1.11 + 1.12) Add a pen (this part will only work if the .dae file is in the same folder as this script!)
        env.reset()
        # Plot a 3-link planar robot from the toolbox
        p3 = models.DH.Planar3()
        p3.q = np.zeros([1, 3])
        # Give the robot a cylinder mesh (links) to display in Swift environment
        cyl_viz = CylindricalDHRobotPlot(p3, cylinder_radius=0.05, multicolor=True)
        p3 = cyl_viz.create_cylinders()
        env.add(p3)

        # Create pen - if have a ply file, convert to DAE (or STL) to use with Swift
        current_dir = os.path.dirname(os.path.abspath(__file__))
        dae_path = os.path.join(current_dir, "pen.dae")
        pen_mesh = Mesh(filename=dae_path)


        # 1.13) Set pen transform to be at robot end-effector and translate it 0.1m along Z
        pen_mesh.T = p3.fkine(p3.q).A @ transl(0, 0, 0.1)
        # Add pen to environment
        env.add(pen_mesh)
        env.set_camera_pose([3, 3, 2], [0, 0, 0])  # (position, look-at)

        input("Press enter to continue\n")
        # 1.14) Move the pen as if it were on the end-effector through a naively-created arm trajectory
        for i in np.arange(-pi/4, pi/4+0.01, 0.01):
            p3.q = [i,i,i]
            env.step(0.05)
            ee_tr = p3.fkine(p3.q)
            pen_mesh.T = ee_tr.A @ transl(0, 0, 0.1)
            # Create a red sphere at each point
            new_point = Sphere(radius=0.025, color=[1.0, 0.0, 0.0, 1.0])
            new_point.T = ee_tr
            env.add(new_point)
            env.step(0.05)

        input("Press enter to finish")
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



# ---------------------------------------------------------------------------------------#
# Main block
if __name__ == "__main__":
    soln = Lab4Solution()
    soln.question1()

    print("Lab 4, Question 1 Solution completed. You can now close the window.")
    plt.close("all")
    time.sleep(0.5)


