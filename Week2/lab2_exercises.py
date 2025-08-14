
# Required libraries
import numpy as np
import matplotlib.pyplot as plt
import time
import keyboard
from scipy import linalg
from spatialmath import SE3
from spatialmath.base import transl, trotx, troty, tr2rpy, r2q
from roboticstoolbox import DHLink, DHRobot
from ir_support import RobotCow, tranimate_custom, place_fence, orthogonalize_rotation

from math import pi

def lab2_exercises_run():
    plt.close("all")
    #input("Press Enter to begin\n")
    lab = Lab2Exercises()
    # Uncomment the function you want to test
    # lab.question1()
    # lab.question1_part9()
    # lab.question2()
    # lab.question3()
    # lab.question3_part8()
    # lab.question3_point8()
    lab.question4()


class Lab2Exercises:
    def __init__(self):
        pass

    def question1(self):
        """
        Question 1: Animate a UAV flying through a sequence of transforms.
        """
        start_time = time.time()
        speed = 60  # Speed in frames per second

        # 1.1)  Start at the origin and move up to 10m off the ground (positive Z)

        tr_start = np.eye(4)
        tr_end = transl([0, 0, 10])  # Move to [0, 0, 10]
        tranimate_custom(tr_start, tr_end, speed=speed)

        # 1.2)  Rotate (roll) around the X axis by -30 degrees so the Y axis is pointing more towards the ground than before

        tr_start = tr_end
        tr_end = transl([0, 0, 10]) @ trotx(-30 * pi / 180)  # Rotate -30 degrees around X
        tranimate_custom(tr_end, tr_end, speed=speed)

        # 1.3)  Move in the direction of global Y to [0,2,10]

        tr_start = tr_end
        tr_end = transl([0, 2, 10]) @ trotx(-30 * pi / 180)  # Maintain the previous orientation and move to [0, 2, 10]
        tranimate_custom(tr_start, tr_end, speed=speed)
        self.display_orientation_text(tr_end)

        # 1.4)  Roll back to level (so the orientation is now eye(3))

        tr_start = tr_end
        tr_end = transl([0, 2, 10]) # removes orientation change from step 2
        tranimate_custom(tr_start, tr_end, speed=speed)

        # 1.5)  Rotate (pitch) around the Y axis by 30 degrees so the X axis is pointing more towards the ground than before

        tr_start = tr_end
        tr_end = transl([0, 2, 10]) @ troty(30 * pi / 180)  # Rotate 30 degrees around Y
        tranimate_custom(tr_start, tr_end, speed=speed)

        # 1.6) Move in the direction of global X to [2,2,10]

        tr_start = tr_end
        tr_end = transl([2, 2, 10]) @ troty(30 * pi / 180)  # Maintain the previous orientation and move to [2, 2, 10]
        tranimate_custom(tr_start, tr_end, speed=speed)

        # 1.7) Roll back to level (so the orientation is now eye(3))

        tr_start = tr_end
        tr_end = transl([2, 2, 10])  # Removes orientation change
        tranimate_custom(tr_start, tr_end, speed=speed)

        # 1.8) Go to the ground so that the new position is [2,2,0]

        tr_start = tr_end
        tr_end = transl([2, 2, 0])  # Move to [2, 2, 0]
        tranimate_custom(tr_start, tr_end, speed=speed)

        # 1.9) Encode the steps 1.1-1.8 in a ‘for’ loop and use the 'fps' option in ‘tranimate’ to speed up the animation

    def question1_part9(self): 

        translations = [
            transl([0, 0, 0]),  # Start at the origin
            transl([0, 0, 10]),  # Step 1.1
            transl([0, 0, 10]) @ trotx(-30 * pi / 180),  # Step 1.2
            transl([0, 2, 10]) @ trotx(-30 * pi / 180),  # Step 1.3
            transl([0, 2, 10]),  # Step 1.4
            transl([0, 2, 10]) @ troty(30 * pi / 180),  # Step 1.5
            transl([2, 2, 10]) @ troty(30 * pi / 180),  # Step 1.6
            transl([2, 2, 10]),  # Step 1.7
            transl([2, 2, 0])   # Step 1.8
        ]

        speed = 60  # Speed in frames per second

        for i in range(len(translations) - 1):
            tr_start = translations[i]
            tr_end = translations[i + 1]
            tranimate_custom(tr_start, tr_end, speed=speed)
            self.display_orientation_text(tr_end) #1.10

        # 1.10)  Use the text tool from Week 1 Lab to plot in the left hand corner the RPY and quaternion value of the orientation at each step



        input("Press Enter to continue\n")

    def question2(self):
        """
        Question 2: Visualise cow herd and simulate movement.
        """
        print("TODO: Instantiate RobotCows and simulate random movements")
        # 2.1 Create an instance of RobotCows

        cow_herd = RobotCow()

        # 2.2 Check the default cow with: cow_herd.num_cows        

        print(f"Number of cows: {cow_herd.num_cows}")

        # 2.3 And plot the random walk movement of them with:
        
        cow_herd.plot_single_random_step()

        # 2.4 Increase the number of cows
        
        plt.close("all")
        cow_herd = RobotCow(10)

        # 2.5 Test many random steps
        
        num_steps = 100
        delay = 0.01
        cow_herd.test_plot_many_step(num_steps, delay)
        #plt.show() # Uncomment this line to keep the figure on

        # 2.6 Query the location of the 2nd cow with: cow_herd.cow_list[1]['base']    

        cow_2_location = cow_herd.cow_list[1]['base'] 
        print(f"Location of the 2nd cow: {cow_2_location}")    
        
        input("Press Enter to continue\n")

    def question3(self):
        """
        Question 3: Combine UAV animation, cow herd, and fences.
        """
        print("TODO: Animate UAV + herd in paddock with fences")

        # 3.1) place_fence at [5,0,0] and [-5,0,0]
        place_fence(position=[5,0,0], orientation= 0, plot_type='surface', scale=[1, 10, 1])
        place_fence(position=[-5,0,0], orientation= 0, plot_type='surface', scale=[1, 10, 1])

        # 3.2) Add more fences at [0,5,0] and [0,-5,0] rotated 90 deg
        place_fence(position=[0,5,0], orientation= 90, plot_type='surface', scale=[1, 10, 1])
        place_fence(position=[0,-5,0], orientation= 90, plot_type='surface', scale=[1, 10, 1])  

        # 3.3) Create a cow herd with more than two cows.

        cow_herd = RobotCow(3)  # Create a herd with 10 cows

        # 3.4) Plot the transformation plot of the UAV starting at the origin (same as question 1)

        translations = [
            transl([0, 0, 0]),  # Start at the origin
            transl([0, 0, 10]),  # Step 1.1
            transl([0, 0, 10]) @ trotx(-30 * pi / 180),  # Step 1.2
            transl([0, 2, 10]) @ trotx(-30 * pi / 180),  # Step 1.3
            transl([0, 2, 10]),  # Step 1.4
            transl([0, 2, 10]) @ troty(30 * pi / 180),  # Step 1.5
            transl([2, 2, 10]) @ troty(30 * pi / 180),  # Step 1.6
            transl([2, 2, 10]),  # Step 1.7
            transl([2, 2, 0])   # Step 1.8
        ]

        speed = 60  # Speed in frames per second

        for i in range(len(translations) - 1):
            tr_start = translations[i]
            tr_end = translations[i + 1]
            tranimate_custom(tr_start, tr_end, speed=speed)
            cow_herd.plot_single_random_step()
            # Display the transfoirmations
            for cow_index in range(cow_herd.num_cows):
                UAV_to_cow_transform = linalg.inv(tr_end) @ cow_herd.cow_list[cow_index]['base']
                print(f"UAV to Cow {cow_index} transformation: {UAV_to_cow_transform}")
            self.display_orientation_text(tr_end)

        # 3.5) Determine the transformation between the UAV and each of the cows

        # 3.6) Each time the UAV moves, also move the cows randomly with:              	
        # cow_herd.plot_single_random_step()

        # 3.7) Fly through the flight path from question 1 and at each of the goal location determine the transformation between
        # the UAV and all the cows

        # 3.8) Create a cow herd with one cow and move your drone so that at each step the cow follows stays 5 meters above
        # it but directly overhead
    def question3_part8(self):

        #create feences and cow heard of 1 cow
        place_fence(position=[5,0,0], orientation= 0, plot_type='surface', scale=[1, 10, 1])
        place_fence(position=[-5,0,0], orientation= 0, plot_type='surface', scale=[1, 10, 1])
        place_fence(position=[0,5,0], orientation= 90, plot_type='surface', scale=[1, 10, 1])
        place_fence(position=[0,-5,0], orientation= 90, plot_type='surface', scale=[1, 10, 1])  
        cow_herd = RobotCow(1)  # Create a herd with 1 cow

        num_steps = 10
  
        tr_start = transl([0, 0, 5]) 

        for i in range(num_steps):
            cow_herd.plot_single_random_step()
            tr_goal = cow_herd.cow_list[0]['base'] @ transl([0, 0, 5]) 
            tranimate_custom(tr_start, tr_goal, speed=60)
            tr_start = tr_goal  # Update start for next iteration

        input("Press Enter to continue\n")

    def question4(self):
        """
        Question 4: Create 3-link manipulator using DH parameters
        """
        # 4.1 Work out the DH Parameters by trial and error

        # 4.2 Generate the robot model with 3 links.

        link1 = DHLink(d=0, a=1, alpha=0, offset=0, qlim=[-pi, pi])
        link2 = DHLink(d=0, a=1, alpha=0, offset=0, qlim=[-pi, pi])
        link3 = DHLink(d=0, a=1, alpha=0, offset=0, qlim=[-pi, pi])
        robot = DHRobot([link1, link2, link3], name ='myRobot')
        q = np.zeros([1,3]) # This creates a vector of n joint angles at 0.
        workspace = [-3, +3, -3, +3, 3,-3 +3]
        robot.plot(q= q, limits= workspace) 

        # 4.3) You can manually play around with the robot:
       	# Close current figure because teach method will create a new one
        plt.close() 
        fig = robot.teach(robot.q, block = False)
        input("Press Enter to play with teach and then press Enter again to finish\n")
        plt.close() # Close current figure because teach method will create a new one 
        fig = robot.teach(robot.q, limits= workspace, block = False)

        while True:
            if keyboard.is_pressed('enter'):
                break
            print("q = ", robot.q)
            fig.step(0.05)

        # 4.5 Get the joint limits
        print("joint limits: \n", robot.qlim)
        
        input("Press Enter to continue\n")

        
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
        

if __name__ == "__main__":
    lab2_exercises_run()
    keyboard.unhook_all()
    plt.close("all")
    time.sleep(0.5)
