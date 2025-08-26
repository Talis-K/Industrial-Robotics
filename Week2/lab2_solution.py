# Require libraries
import numpy as np
import matplotlib.pyplot as plt
import time
import keyboard
from scipy import linalg
from spatialmath import SE3
from spatialmath.base import transl, trotx, troty, tr2rpy, r2q
from roboticstoolbox import DHLink, DHRobot
from ir_support import RobotCow, tranimate_custom, place_fence, orthogonalize_rotation

# Useful variables
from math import pi

def lab2_solution_run():
    plt.close("all")
    #input("Press Enter to begin\n")
    lab2_solution = Lab2Solution()
    # lab2_solution.question1()
    # lab2_solution.question1_as_for_loop()
    # lab2_solution.question2()
    # lab2_solution.question3()
    # lab2_solution.question3_point8()
    lab2_solution.question4()


# ---------------------------------------------------------------------------------------#
class Lab2Solution:
    def __init__(self):
        # No generic initialisation required
        pass

    # ---------------------------------------------------------------------------------------#
    def question1(self):
        """
        Question 1: Animate transform (Quad copter flying)
        """
        start_time = time.time()
        speed = 60 # Speed of the animation
        # 1.1
        tr_start = np.eye(4)
        tr_end = transl([0,0,10])
        tranimate_custom(tr_start, tr_end, speed=speed)
        self.display_orientation_text(tr_end) # 1.10)
    
        # 1.2
        tr_start = transl([0,0,10])
        tr_end = transl([0,0,10]) @ trotx(-30*pi/180)
        tranimate_custom(tr_start, tr_end, speed=speed)
        self.display_orientation_text(tr_end) # 1.10)
        # 1.3
        tr_start = tr_end
        tr_end = transl([0,2,10]) @ trotx(-30 * pi/180)
        tranimate_custom(tr_start, tr_end, speed=speed)
        self.display_orientation_text(tr_end) # 1.10)
        # 1.4
        tr_start = tr_end
        tr_end = transl([0,2,10])
        tranimate_custom(tr_start, tr_end, speed=speed)
        self.display_orientation_text(tr_end) # 1.10)
        # 1.5
        tr_start = tr_end
        tr_end = transl([0,2,10]) @ troty(30 * pi/180)
        tranimate_custom(tr_start, tr_end, speed=speed)
        self.display_orientation_text(tr_end) # 1.10)
        # 1.6
        tr_start = tr_end
        tr_end = transl([2,2,10]) @ troty(30 * pi/180)
        tranimate_custom(tr_start, tr_end, speed=speed)
        self.display_orientation_text(tr_end) # 1.10)
        #1.7
        tr_start = tr_end
        tr_end = transl([2,2,10])
        tranimate_custom(tr_start, tr_end, speed=speed)
        self.display_orientation_text(tr_end) # 1.10)
        # 1.8
        tr_start = tr_end
        tr_end = transl([2,2,0])
        tranimate_custom(tr_start,tr_end,speed= speed, hold= True)
        self.display_orientation_text(tr_end) # 1.10)
        # 1.9 
        #is already done above

        end_time = time.time()
        print("Ex1 took: {:.2f} seconds.".format(end_time - start_time))

        input("Finished question 1, press Enter to continue\n")
    
    def question1_as_for_loop(self):
        """
        Question 1: Animate transform (Quadcopter flying)
        """
        start_time = time.time()
        speed = 20  # Animation speed

        transforms = [
            transl(0, 0, 10),                                      # 1.1
            transl(0, 0, 10) @ trotx(-30 * pi / 180),              # 1.2
            transl(0, 2, 10) @ trotx(-30 * pi / 180),              # 1.3
            transl(0, 2, 10),                                      # 1.4
            transl(0, 2, 10) @ troty(30 * pi / 180),               # 1.5
            transl(2, 2, 10) @ troty(30 * pi / 180),               # 1.6
            transl(2, 2, 10),                                      # 1.7
            transl(2, 2, 0)                                        # 1.8
        ]
        
        tr_start = np.eye(4)

        tranimate_custom(tr_start, transl(0, 0, 10), speed=100)
        input("Press Enter to visualize the transforms\n")

        for i, tr_end in enumerate(transforms):
            tranimate_custom(tr_start, tr_end, speed=speed, hold=(i == len(transforms) - 1))
            self.display_orientation_text(tr_end)  # 1.10
            tr_start = tr_end

        print(f"Ex1 took: {time.time() - start_time:.2f} seconds.")
        input("Finished question 1, press Enter to continue\n")

    # ---------------------------------------------------------------------------------------#
    def question2(self):
        """
        Question 2 (done before 1 so it can keep this all in one file)
        """
        # 2.1 Create an instance of the cow herd with default parameters
        cow_herd = RobotCow()
        # 2.2 Check how many cows there are
        print("Number of cows: ", cow_herd.num_cows)
        # 2.3 Plot on single iteration of the random step movement
        input("Press Enter to visualize a single step\n")
        cow_herd.plot_single_random_step()
        plt.pause(0.01)
        input('Finished question 2.3, press Enter to continue\n')
        # 2.4 Create a new instance with 10 cows
        plt.close("all")
        cow_herd = RobotCow(10)
        plt.pause(0.01)
        # 2.5 Test many random steps
        num_steps = 100
        delay = 0.01
        cow_herd.test_plot_many_step(num_steps,delay)
        input("Press Enter to continue\n")
        # 2.6 Query the location of the 2nd cow
        try:
            print("Location of 2nd cow:\n", SE3(cow_herd.cow_list[1]['base']))
        except ValueError: 
            # This will be raised if the SE3 doesn't recognise the input 
            # as a valid SE3 matrix due to accumulation error of the matrix multiplication process. 
            # Specifically, the rotation matrix of the input doesn't keep its orthogonality.
            # We will try to orthogonalize the rotation matrix of the input
            cow_herd.cow_list[1]['base'][:3,:3] = orthogonalize_rotation(cow_herd.cow_list[1]['base'][:3,:3])
            print("Fix orthogonality error. Location of 2nd cow:\n", SE3(cow_herd.cow_list[1]['base']))

        input('Finished Question 2, press Enter to continue\n')

    # ---------------------------------------------------------------------------------------#
    def question3(self):
        """
        Question 3: Combine question 1 with question 2
        """
        plt.close()
        # 3.1-3.2 Place fence into the enviroment
        place_fence(position=[5,0,0], orientation= 0, plot_type='surface', scale=[1, 10, 1])
        place_fence(position=[-5,0,0], orientation= 0, plot_type='surface', scale=[1, 10, 1])
        place_fence(position=[0,5,0], orientation= 90, plot_type='surface', scale=[1, 10, 1])
        place_fence(position=[0,-5,0], orientation= 90, plot_type='surface', scale=[1, 10, 1])        
        # plt.pause(10)

        # 3.3 Create a cow herd with more than 2 cows
        cow_herd = RobotCow(3)
        # 3.4 Plot the transform of the UAV starting at the origin
        uav_tr = [None for _ in range(9)] # List to store all nine UAV's transforms
        uav_tr[0] = np.eye(4)
        # trplot(uav_tr[0])
        tranimate_custom(uav_tr[0], uav_tr[0], speed= 100, hold = True)
        
        plt.pause(0.01)
        # 3.5 Determine the transform between the UAV and each of the cows
        for cow_index in range(cow_herd.num_cows):
            print("- At trajectory step",1,"the UAV TR to cow",cow_index,"is:")
            try:
                print(SE3(linalg.inv(uav_tr[0]) @ cow_herd.cow_list[cow_index]['base']))
            except ValueError:
                cow_herd.cow_list[cow_index]['base'][:3,:3] = orthogonalize_rotation(cow_herd.cow_list[cow_index]['base'][:3,:3])
                print(f"Fix orthogonality error.\n", SE3(cow_herd.cow_list[cow_index]['base']))
            
        cow_herd.plot_single_random_step()
        plt.pause(0.01)
        
        # 3.6-3.7 Fly through Question 1, at each time the UAV moves to a goal, the
        # cows move randomly once, then determine the transform between the UAV and
        # all the cows
        uav_tr[1] = transl([0,0,10])
        uav_tr[2] = transl([0,0,10]) @ trotx(-30 * pi/180)
        uav_tr[3] = transl([0,2,10]) @ trotx(-30 * pi/180)
        uav_tr[4] = transl([0,2,10])
        uav_tr[5] = transl([0,2,10]) @ troty(30 * pi/180)
        uav_tr[6] = transl([2,2,10]) @ troty(30 * pi/180)
        uav_tr[7] = transl([2,2,10])
        uav_tr[8] = transl([2,2,0]) 

        plot_range = [-5, 5, -5, 5, 0, 10] # Plot range to visualise the cow herd and the UAV
        for trajectory_step in range(len(uav_tr)-1):
            tranimate_custom(uav_tr[trajectory_step], uav_tr[trajectory_step+1], 
                             speed= 80, dim= plot_range, hold = True)
                            # set 'hold' = True to keep everything displayed 
                            # on current axes not being deleted
                            # trade-off is the uav's trace at each step will be kept
            cow_herd.plot_single_random_step()
            for cow_index in range(cow_herd.num_cows):
                print("- At trajectory step",trajectory_step+1,"the UAV TR to cow",cow_index,"is:")
                try:
                    print(SE3(linalg.inv(uav_tr[trajectory_step]) @ cow_herd.cow_list[cow_index]['base']))
                except ValueError:
                    cow_herd.cow_list[cow_index]['base'][:3,:3] = orthogonalize_rotation(cow_herd.cow_list[cow_index]['base'][:3,:3])
                    print(f"Fix orthogonality error:\n", SE3(cow_herd.cow_list[cow_index]['base']))

        input('Finished question 3.7, press Enter to continue\n')
    # ---------------------------------------------------------------------------------------#
    def question3_point8(self):
        """
        3.8 Create a cow herd with 1 cow and move your drone so that at each 
        step the cow moves stay 5 meters above it but directly overhead
        """
        plt.close()
        cow_herd = RobotCow(1)
        start_time = time.time()
        plot_range = [-5, 5, -5, 5, 0, 10] # Plot range to visualise the cow herd and the UAV

        # Animate drone to go over cow position
        uav_tr_start = transl(0,0,5)
        uav_tr_goal = uav_tr_start @ cow_herd.cow_list[0]['base']
        tranimate_custom(uav_tr_start, uav_tr_goal, speed= 50, dim= plot_range, hold = True) 
        
        # Go through 10 steps (not specified but this is arbitrary)
        num_steps = 10
        for _ in range(num_steps):
            cow_herd.plot_single_random_step()
            uav_tr_start = uav_tr_goal
            uav_tr_goal = cow_herd.cow_list[0]['base'] @ transl(0,0,5)
            tranimate_custom(uav_tr_start, uav_tr_goal, speed = 80, dim= plot_range, hold = True) # Set hold = True to see the trace of the drone
        
        end_time = time.time()
        print("Following the cow took {:.2f} seconds.".format(end_time - start_time))
        input('Finished Question 3.8, press Enter to continue\n') 

    # ---------------------------------------------------------------------------------------#
    def question4(self):
        """
        Question 4 Derive the DH parameters for the simple 3 link manipulator provided. 
        Use these to generate a  model of the manipulator using the Robot Toolbox in MATLAB
        """
        plt.close()

        # 4.1 and 4.2: Define the DH Parameters to create the Kinematic model
        link1 = DHLink(d= 0.5, a= 0.3, alpha= np.deg2rad(90), qlim= [-pi, pi]) 
        link2 = DHLink(d= 0, a= 1, alpha= 0, qlim= [-pi, pi]) 
        link3 = DHLink(d= 0, a= 0.2, alpha= -np.deg2rad(90), qlim= [-pi, pi]) 
        link4 = DHLink(d= 0.7, a= 0, alpha= np.deg2rad(90), qlim= [-pi, pi])
        robot = DHRobot([link1, link2, link3, link4], name= 'myRobot')
        workspace = [-3, 3, -3, 3, -3, 3]
        # q =  np.zeros([1,4]) # Initial joint angles = 0
        q = np.array([-0.7506, 0.5895, -1.8286,  2.5971])  # Initial joint angles as shown in Canvas

        try:
            options = {"eelength": 1.0, "jointaxislength": 0.5}
            robot.plot(q= q, limits= workspace,options=options)
        except Exception as e:
            print("[Warning] Custom plotting options (e.g., eelength) are not supported unless you patch the Robotics Toolbox.")
            print("See Canvas for instructions on how to apply the patch to enable these visual customisations.")
            robot.plot(q= q, limits= workspace)

        # 4.3 Manually play around with the robot
        input("Press Enter to play with teach and then press Enter again to finish\n")
        plt.close() # Close current figure because teach method will create a new one 
        fig = robot.teach(robot.q, limits= workspace, block = False)

        # 4.4 Get the current joint angles based on the position in the model
        while True:
            if keyboard.is_pressed('enter'):
                break
            print("q = ", robot.q)
            fig.step(0.05)

        # 4.5 Get the joint limits
        print("joint limits: \n", robot.qlim)

    # ---------------------------------------------------------------------------------------#
    @staticmethod
    def display_orientation_text(T, position=(-5, 11), fontsize=10):
        """
        Display RPY and quaternion values in the top-left corner of the plot.

        Parameters:
        - T: SE3 or 4x4 numpy array representing the transform
        - position: tuple (x, y) indicating text location
        - fontsize: font size for the text
        """
        from spatialmath.base import tr2rpy, r2q
        import matplotlib.pyplot as plt
        from math import pi

        rpy_deg = np.round(tr2rpy(T), 2) * 180 / pi
        quat = np.round(r2q(T[:3, :3]), 2)

        msg = f"RPY (deg): {rpy_deg.tolist()}\nQuat: {quat.tolist()}"        

        # Remove previous if it exists
        ax = plt.gca()
        if hasattr(ax.figure, '_orientation_text'):
            try:
                ax.figure._orientation_text.remove()
            except Exception:
                pass

        ax.figure._orientation_text = ax.text(position[0], position[1], 1, msg,
                                            fontsize=fontsize, color='black', verticalalignment='top')
        plt.pause(0.01)
        
# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    lab2_solution_run()
    keyboard.unhook_all() 
    plt.close("all")
    time.sleep(0.5)       