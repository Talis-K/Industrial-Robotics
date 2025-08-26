# Require libraries
import numpy as np
import matplotlib.image as mpimg
from scipy import linalg
import matplotlib.pyplot as plt
from spatialmath import SE2
from spatialmath.base import trplot2 
from pathlib import Path
from ir_support import functions as ir

# Useful variables
from math import pi   

def lab1_solution_run():
    plt.close("all")
    lab1_solution = Lab1Solution()
    # lab1_solution.question1()
    # lab1_solution.question2()
    lab1_solution.questions3_and_4()

    # Uncomment this line below to keep the figure after the last question finished
    # plt.show() 

# ---------------------------------------------------------------------------------------#
class Lab1Solution:
    def __init__(self):
        image_path = Path(__file__).parent / "../Images/Lab1CircularRaceTrack.jpg"
        self.img = mpimg.imread(str(image_path))

        self.fig = None
        self.next_question = False  # Flag can be used to move to the next question (e.g. if closing a window)

        # Constants        
        # Track radii (in pixels) based on measurements from image:
        self.RADIUS_OUTER = (550 - 66)/2   # outer lane: track edge to center
        # => Approx circumference = 2πr ≈ 2π × 242 ≈ 1521 pixels
        self.RADIUS_INNER = (500 - 125)/2  # inner lane: track edge to center
        # => Approx circumference = 2πr ≈ 2π × 187.5 ≈ 1178 pixels                        

    # ---------------------------------------------------------------------------------------#
    def question1(self):
        print('Download and setup the Robotics Toolbox in Python. See videos and links in Canvas.')
        input("Press Enter to continue")

    # ---------------------------------------------------------------------------------------#
    def question2(self):
        self.fig = plt.figure() # create a new figure
        self.fig.canvas.mpl_connect('close_event', self.on_close) # attach a close event to this window
        self.fig.canvas.manager.set_window_title('Question 2')  # set the window title of the figure
        plt.imshow(self.img)
        
        car1_tr = SE2(300, 550, 0)     # initial pose of car 1
        trplot2(car1_tr.A, frame = '1', color = 'b', length = 50, width = 0.05)

        total_steps = 360 # steps per revolulation
    
        # So the transform each step is
        car1_move_tr = SE2((2 * pi * self.RADIUS_OUTER)/total_steps, 0, 0)
        car1_turn_tr = SE2(0, 0, -2*pi/total_steps)

        for _ in range(total_steps): 
            # Update figure
            plt.cla()  # Clear the current axes
            plt.imshow(self.img)

            # Update car1 pose: move forward, then turn slightly (counter-clockwise)
            car1_tr = ir.clean_SE2(car1_tr * car1_move_tr * car1_turn_tr)
            trplot2(car1_tr.A, frame = '1', color = 'b', length = 50, width = 0.05) 

            message = '\n'.join(['  '.join([f"{val:} " for val in row]) for row in car1_tr.A])
            plt.text(10, 50, message, fontsize=10, color=[.6, .2, .6])

            if _ == 105:
                print(message)  # Print the pose of car1 at step 95

            plt.draw()
            plt.pause(0.01)

            # Move to the next question if prompted
            if self.next_question:
                self.next_question = False # Reset the flag
                break   # Exit the loop

    # ---------------------------------------------------------------------------------------#
    def questions3_and_4(self):
        for question in [3, 4]: # question will take value 3 and 4
            self.fig = plt.figure() # create a new figure
            self.fig.canvas.manager.set_window_title('Question ' + str(question))  # Set the window title of the figure
            self.fig.canvas.mpl_connect('close_event', self.on_close) # attach a close event to this window
            if question == 4:
                plt.subplot(1, 2, 1)
                
            plt.imshow(self.img)

            car1_tr = SE2(300, 550, 0)     # initial pose of car 1
            car2_tr = SE2(508, 300, np.deg2rad(90))     # initial pose of car 2

            # For distance plot (i.e. question 4)
            if question == 4:
                plt.subplot(1,2,2)
                plt.xlabel('Timestep')
                plt.ylabel("Sensor reading - distance between cars")
            
            total_steps = 360
            # So the transform each step is
            car1_move_tr = SE2((2 * pi * self.RADIUS_OUTER)/total_steps, 0, 0)
            car1_turn_tr = SE2(0, 0, -2*pi/total_steps)
            car2_move_tr = SE2((2 * pi * self.RADIUS_INNER)/total_steps, 0, 0)
            car2_turn_tr = SE2(0, 0, 2*pi/total_steps)
            dist = np.zeros(total_steps)

            for i in range(total_steps): 
                car1_tr = ir.clean_SE2(car1_tr * car1_move_tr * car1_turn_tr)
                car2_tr = ir.clean_SE2(car2_tr * car2_move_tr * car2_turn_tr)

                # print("car1_to_2_tr = \n", car1_tr.inv() * car2_tr)
                # print("car2_to_1_tr = \n", car2_tr.inv() * car1_tr)

                if i == 4:
                    car1 = car1_tr
                
                if i == 6:
                    car2 = car2_tr
                    car1_to_car2_tr = car1_tr.inv() * car2_tr
                    matrix_str = '\n'.join(['  '.join([f"{val:.2f}" for val in row]) for row in car1_to_car2_tr.A])
                    print("car1_to_car2_tr =\n", matrix_str)


                if question == 4: # Switch to update subplot 1
                    plt.subplot(1,2,1)
                
                plt.cla()                
                plt.imshow(self.img)
 
                trplot2(car1_tr.A, frame = '1', color = 'b', length = 50, width = 0.05)
                trplot2(car2_tr.A, frame = '2', color = 'r', length = 50, width = 0.05)

                if question == 4:
                    plt.subplot(1,2,2)
                    plt.xlabel('Timestep')
                    plt.ylabel("Sensor reading - distance between cars")
                    
                    dist[i] = linalg.norm(car1_tr.t - car2_tr.t)
                    dist_point_h = plt.plot(range(1, i+1), dist[:i], 'b-')
                
                plt.draw()
                plt.pause(0.01)

                # Move to the next question if prompted
                if self.next_question:
                    self.next_question = False # Reset the flag
                    break   # Exit the loop

    # ---------------------------------------------------------------------------------------#
    # This function here will be executed when a figure is closed
    def on_close(self, event):
        self.next_question = True
    
# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    lab1_solution_run()