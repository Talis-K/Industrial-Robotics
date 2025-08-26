# Require libraries
import numpy as np
import matplotlib.pyplot as plt
import keyboard
import time
from spatialmath.base import *
from spatialmath import SE3
from roboticstoolbox import DHLink, DHRobot

# Useful variables
from math import pi

# ---------------------------------------------------------------------------------------#
def lab3_starter():
    """
    lab3_starter Create a Puma 560 DH SerialLink Model
    Although there is a Puma 560 Model in the toolbox, this code creates a model 
    from the basic DH parameters that we derived in the previous theory exercise.  

    DHLink(theta=__, d=__, a=__, alpha=__, offset=__, qlim =[ ... ])

    """
        
    link1 = DHLink(d=0, a=0, alpha=pi/2, offset=pi/2)

    link2 = DHLink(d=0, a=0.4318, alpha=0, offset=0)

    link3 = DHLink(d=0.15, a=0.0203, alpha=-pi/2, offset=0)

    link4 = DHLink(d=0.4318, a=0, alpha=pi/2, offset=0)

    link5 = DHLink(d=0, a=0, alpha=-pi/2, offset=0)

    link6 = DHLink(d=0, a=0, alpha=0, offset=0)

    my_robot = DHRobot([link1, link2, link3, link4, link5, link6], name= 'Puma560')

    q = np.zeros([1,6])

    my_robot.plot(q)

    print("Gravity:\n",my_robot.gravity)
    print("Base:\n",my_robot.base)
    print("Tool:\n",my_robot.tool)

    input("Enter to continue\n")
    
    # Assign a random config
    my_robot.q = np.random.rand(1,6)
    tr_ee = my_robot.fkine(my_robot.q)
    
    # Inverse kinematic
    q_result = my_robot.ik_LM(tr_ee)[0]
    print("Actual configuration:   ", my_robot.q, "\n"
          "IK Solver configuration:", q_result)
    
    print(my_robot.q - q_result < np.finfo(float).eps)

    ## Teach
    input("Enter to play with teach and then hit Enter again to finish\n")
    plt.close()
    fig = my_robot.teach(my_robot.q, block= False)
    while True:
        if keyboard.is_pressed('enter'):
            break
        fig.step(0.05)

    t_f1 = 0.0 # Total time for forward kinematic calculation using fkine
    t_f2 = 0.0 # Total time for forward kinematic calculation doing manually

    for i in np.arange(0, 1+0.001, 0.001):
        q =[i,i,i,i,i,i]

        ## fkine
        t0 = time.time()
        fkine_tr = my_robot.fkine(q) # use operator .A to get the NDArray transform
        t1 = time.time()

        ## Manual calculation of link transforms
        base_tr = np.eye(4)
        joint_0to1_tr = get_joint_to_joint_tr(q[0],0,     0,      pi/2)
        joint_1to2_tr = get_joint_to_joint_tr(q[1],0,     0.4318, 0)
        joint_2to3_tr = get_joint_to_joint_tr(q[2],0.15,  0.0203, -pi/2)
        joint_3to4_tr = get_joint_to_joint_tr(q[3],0.4318,0,      pi/2)
        joint_4to5_tr = get_joint_to_joint_tr(q[4],0,     0,      -pi/2)
        joint_5to6_tr = get_joint_to_joint_tr(q[5],0,     0,      0) 
        tool_tr = np.eye(4)
        my_fkine_tr = base_tr @ joint_0to1_tr @ joint_1to2_tr @ joint_2to3_tr @ joint_3to4_tr @ joint_4to5_tr @ joint_5to6_tr @ tool_tr
        t2 = time.time()

        print("Foward kinematics by fkine:\n", fkine_tr)
        print("Manual forward kinematics:\n", SE3(my_fkine_tr))
        # time.sleep(0.05)

        t_f1 += t1 - t0
        t_f2 += t2 - t1

    print("Total time for forward kinematic calculation using fkine:", t_f1)
    print("Total time for forward kinematic calculation doing manually:", t_f2)

def get_joint_to_joint_tr(q,d,a,alpha):
    """
    Get joint to joint transform
    """
    return trotz(q) @ transl([0,0,d]) @ transl([a,0,0]) @ trotx(alpha)

# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    lab3_starter()
    