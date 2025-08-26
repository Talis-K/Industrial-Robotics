# Require libraries
from math import sin, cos, atan2, sqrt
import numpy as np
import matplotlib.pyplot as plt
import keyboard
from spatialmath.base import *
from roboticstoolbox import models, jtraj

# Useful variables
from math import pi

# ---------------------------------------------------------------------------------------#
def lab4_starter():
    plt.close("all")
    
    ## Create robot and workspace
    two_link = models.DH.TwoLink()
    
    # specify workspace
    workspace = [-1, 2.5, -2, 2, -1, 2]
    
    qz = [0,0]
    two_link.plot(qz, limits= workspace)

    input("Press Enter to play with teach and then press Enter again to finish\n")
    plt.close()

    fig = two_link.teach(two_link.q, block= False)
    while not keyboard.is_pressed('enter'):
        fig.step(0.05)
        q = two_link.q
    
    plt.close()
    fig = two_link.plot(qz, limits= workspace)
    print("Press Enter to continue!")
    hold()
    
    ## Calculate joint angles for a two link planar arm
    # specify an end effector position
    tr = [1.7, 0, 0.1]
    x = tr[0]
    z = tr[2]

    # using cosine rule (a^2 = b^2 + c^2 - 2bc*cos(A)), 
    # where cos(pi-A) = -cos(A)
    # and cosA = cos(A) 
    # and x^2+z^2 = a^2 
    # and a and b are the link lengths equal to 1m
    # in Python, '**' operator is used for power operation, while '^' is reserved for XOR bitwise operation
    cos_A = (x**2 + z**2 - 1**2 - 1**2)/(2*1*1)
    
    # calculate joint 2 for poses 1 and 2
    pose1_theta2 = atan2(sqrt(1 - cos_A**2),cos_A)
    pose2_theta2 = atan2(-sqrt(1 - cos_A**2),cos_A)

    # calculate joint 1 for poses 1 and 2
    pose1_theta1 = atan2(z,x) - atan2((1) * sin(pose1_theta2), 1 + 1*cos(pose1_theta2))
    pose2_theta1 = atan2(z,x) - atan2((1) * sin(pose2_theta2), 1 + 1*cos(pose2_theta2))

    pose1 = [pose1_theta1, pose1_theta2]
    pose2 = [pose2_theta1, pose2_theta2]

    # Plot two configurations
    fig.ax.cla()
    two_link.plot(pose1, limits= workspace)

    print("Press Enter to continue!")
    hold()

    fig = two_link.plot(pose2, limits= workspace)
    
    ## Confirm joint angles using fkine
    q_fkine1 = two_link.fkine(pose1)
    q_fkine2 = two_link.fkine(pose2)
    print(np.abs(q_fkine1 - q_fkine2) <= np.finfo('float').eps)

    print("Press Enter to continue!")
    hold()
    
    ## Draw straight line
    # Plot a trajectory for the end-effector which moves from x = 1.7 to x = 0.7
    # while maintaining a z height of 0.1
    z = 0.1

    # preallocate matrix if you want to view the joint angles later
    q_matrix = np.zeros([100,2])
    count = 0

    # # Can also try the following joint angle guess into ikine_LM as an example to
    # # how the manipulator motion changes based on the given 'q0' value.
    # q0 = [pi/3, -2*pi/3]
    # new_q = two_link.ikine_LM(transl(tr), q0 = q0, mask = [1,1,0,0,0,0]).q

    for x in np.arange(1.7, 0.7-0.05, -0.05):
        cos_A = (x**2 + z**2 - 1**2 - 1**2)/(2*1*1)
        q_matrix[count,1] = (atan2(-sqrt(1-cos_A**2),cos_A))
        q_matrix[count,0] = (atan2(z,x)-atan2((1)*sin(q_matrix[count,1]),1+(1)*cos(q_matrix[count,1])))
        two_link.q = q_matrix[count,:]
        fig.step(0.05) # set a refresh rate of 20Hz to visualize the animation

        # # dot q to take out the joint angle value of the IKSolution object returned by ikine_LM
        # new_q= two_link.ikine_LM(transl(x,0,z), q0= new_q, mask= [1,1,0,0,0,0]).q 
        # two_link.q = new_q
        # fig.step(0.05)

        print(f'Step {count}. Fkine solution:\n', two_link.fkine(q_matrix[count,:])) # Note that we don't exactly reach the goal tr 
        # two_link.fkine(new_q) # Note that we don't exactly reach the goal tr
        count += 1

    print("Enter to continue!")
    hold()

    ## Comparing the jtraj trajectory generation method
    q_matrix = q_matrix[0:count,:] # resize to be only the used rows
    # dot q to get the list of the joint angle values of the Trajectory object returned by jtraj
    jtraj_q_matrix = jtraj(q_matrix[0,:], q_matrix[-1,:], np.size(q_matrix,0)).q
    jtraj_end_effector_points = []
    for q in jtraj_q_matrix:
        two_link.q = q
        trplot(two_link.fkine(q).A, color= 'b')
        jtraj_end_effector_points.append(transl(two_link.fkine(q).A))
        fig.step(0.05)
    jtraj_end_effector_points = np.array(jtraj_end_effector_points)

    print("Enter to continue!")
    hold()
    
    invkin_end_effector_points = []
    for q in q_matrix:
        two_link.q = q
        trplot(two_link.fkine(q).A, color= 'r')
        invkin_end_effector_points.append(transl(two_link.fkine(q).A))
        fig.step(0.05)
    invkin_end_effector_points = np.array(invkin_end_effector_points)

    ## 2D view of the end-effector path. Note that z is not fixed
    plt.figure('Plot 2D')
    ax = plt.subplot()
    ax.plot(jtraj_end_effector_points[:,0], jtraj_end_effector_points[:,2], 'b*-')
    ax.plot(invkin_end_effector_points[:,0], invkin_end_effector_points[:,2], 'r*-')
    ax.axis('equal')

    ax.legend(['jtraj_end_effector_points','invkin_end_effector_points'])
    # Set labels for the axes
    ax.set_xlabel('x(m)')
    ax.set_ylabel('z(m)')
    
    print("Enter to Exit!\n")
    hold()

def hold():
    while not keyboard.is_pressed('enter'):
        plt.pause(0.05)

# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    lab4_starter()
    