import numpy as np
import matplotlib.image as mpimg
from scipy import linalg
import matplotlib.pyplot as plt
from spatialmath import SE2
from spatialmath.base import trplot2
from pathlib import Path
from ir_support import functions as ir
from math import pi


class lab1Exercise:
     def __init__(self):
       # Load trackl image
       plt.close('all')
       self.image_path = Path("Images\Lab1CircularRaceTrack.jpg").parent / "Lab1CircularRaceTrack.jpg"
       self.img = mpimg.imread(str(self.image_path))

       #Set constrains, track radii
       self.RADIUS_OUTER = 550 - 66   # outer lane: track edge to center = 484
       self.RADIUS_INNER = 500 - 125  # inner lane: track edge to center = 375
    
     def run(self):
       #self.qestion1()
       #self.question2()
       self.question3and4()
    
     def qestion1(self):
       
       print("download and install all toolbaxes")
       #input("Press Enter to continue...")

     def question2(self):

       #track image
       fig = plt.figure()
       fig.canvas.manager.set_window_title('Q2')   
       plt.imshow(self.img)
       
       car1 = SE2(300, 550, 0)  # Initial position of car1
       trplot2(car1.A, frame='1', length=50, color='b', width = 0.05)

       num_steps = 360  # Number of incremental steps for one full circle (360 degrees)

       #trabsorfms are 
       car1_move = SE2((pi* self.RADIUS_OUTER)/num_steps, 0, 0)  # Move forward in the outer lane
       car1_turn = SE2(0, 0, -2 * np.pi / num_steps)

       for i in range(num_steps):
           plt.cla()  # Clear the current axis
           plt.imshow(self.img)  # Redraw the image
          
           #update car pose
           car1 = ir.clean_SE2(car1*car1_move*car1_turn)
           trplot2(car1.A, frame='1', length=50, color='b', width = 0.05)
           message = '\n'.join(['   '.join([f"{val:.2g}" for val in row]) for row in car1.A])
           plt.text(10, 50, message, fontsize=10, color=(0.6, 0.2, 0.6))
           plt.draw()
           plt.pause(0.01)


     def question3and4(self):
        car2 = SE2(300, 125, 0)
        trplot2(car2.A, frame='2', length=50, color='r', width = 0.05)
        num_steps = 360
        car2_move = SE2((pi * self.RADIUS_INNER) / num_steps, 0, 0)
        car2_turn = SE2(0, 0, 2 * np.pi / num_steps)

        car1 = SE2(300, 550, 0)  # Initial position of car1
        trplot2(car1.A, frame='1', length=50, color='b', width = 0.05)
        car1_move = SE2((pi* self.RADIUS_OUTER)/num_steps, 0, 0)  # Move forward in the outer lane
        car1_turn = SE2(0, 0, -2 * np.pi / num_steps)

        dist = np.zeros(num_steps)

        for i in range(num_steps):
           
           plt.subplot(1, 2, 1)
           plt.cla()  # Clear the current axis
           plt.imshow(self.img)  # Redraw the image
          
           #update car pose
           car1 = ir.clean_SE2(car1*car1_move*car1_turn)
           trplot2(car1.A, frame='1', length=50, color='b', width = 0.05)
           message = '\n'.join(['   '.join([f"{val:.2g}" for val in row]) for row in car1.A])
           plt.text(10, 50, message, fontsize=10, color=(0.6, 0.2, 0.6))

           car2 = ir.clean_SE2(car2*car2_move*car2_turn)
           trplot2(car2.A, frame='2', length=50, color='r', width = 0.05)
           message = '\n'.join(['   '.join([f"{val:.2g}" for val in row]) for row in car2.A])
           plt.text(10, 150, message, fontsize=10, color=(0.6, 0.2, 0.6))    

           car1_to_car2_tr = car1.inv() * car2
           car2_to_car1_tr = car2.inv() * car1
           print ("Transformation from car1 to car2: \n", car1_to_car2_tr)
           print ("Transformation from car2 to car1: \n", car2_to_car1_tr)

           plt.subplot(1, 2, 2)
           plt.xlabel("Timestep")
           plt.ylabel("Distance")
           dist[i] = linalg.norm(car1_to_car2_tr.t - car2_to_car1_tr.t)
           dist_poiny_h = plt.plot(range(1, i+1), dist[:i], 'b-')
           plt.draw()
           plt.pause(0.01)


        

if __name__ == "__main__":
   lab = lab1Exercise()
   lab.run()

