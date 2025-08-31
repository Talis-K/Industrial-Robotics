import spatialgeometry as geometry
import numpy as np
import swift
import time
import os
from ir_support import UR3, RectangularPrism, line_plane_intersection
from itertools import combinations
from roboticstoolbox import jtraj
from spatialmath.base import transl
from spatialmath import SE3
from spatialgeometry import Cuboid, Mesh, Sphere
from math import pi
    
class LabAT1():
    def do_it(self):
        
        steps = 50                                                                                                             # specify no. of steps
        # specify poses of bricks
        brick_pose = np.array([SE3(-0.2, 0, 0),                                                                                 # pose of brick 1
                               SE3(-0.2, 0.2, 0),                                                                               # pose of brick 2
                               SE3(-0.2, -0.2, 0),                                                                              # pose of brick 3
                               SE3(-0.3, 0, 0),                                                                                 # pose of brick 4  
                               SE3(-0.3, 0.2, 0),                                                                               # pose of brick 5
                               SE3(-0.3, -0.2, 0),                                                                              # pose of brick 6
                               SE3(-0.4, 0, 0),                                                                                 # pose of brick 7    
                               SE3(-0.4, 0.2, 0),                                                                               # pose of brick 8      
                               SE3(-0.4, -0.2, 0)])                                                                             # pose of brick 9    
        # specify poses of wall bricks                      
        wall_pose = np.array([SE3(0.3, 0.15, 0),                                                                                # pose of wall brick 1
                                SE3(0.3, 0, 0),                                                                                 # pose of wall brick 2              
                                SE3(0.3, -0.15, 0),                                                                             # pose of wall brick 3
                                SE3(0.3, 0.15, 0.05),                                                                           # pose of wall brick 4
                                SE3(0.3, 0, 0.05),                                                                              # pose of wall brick 5
                                SE3(0.3, -0.15, 0.05),                                                                          # pose of wall brick 6
                                SE3(0.3, 0.15, 0.1),                                                                            # pose of wall brick 7
                                SE3(0.3, 0, 0.1),                                                                               # pose of wall brick 8
                                SE3(0.3, -0.15, 0.1)])                                                                          # pose of wall brick 9
        
        #Create a ground 
        ground_lwh = [3, 3, 0.01]
        ground_center = [0, 0, 0]
        ground_pose = SE3(ground_center)
        ground = Cuboid(scale=ground_lwh, color=[0.8, 0.8, 0.5, 1])
        ground.T = ground_pose

        vertices, faces, faces_normal = RectangularPrism(ground_center[0], ground_center[1], ground_center[2], center = ground_center). get_data()

        #Create a rails
        rail1 = Cuboid(scale=[0.03, 1, 0.02], pose=SE3(0.05, -0.5, 0.015), color=[0.5, 0.5, 0.5, 1], collision=True)  
        rail2 = Cuboid(scale=[0.03, 1, 0.02], pose=SE3(-0.05, -0.5, 0.015), color=[0.5, 0.5, 0.5, 1], collision=True)
        #Create a fence
        fence1 = Cuboid(scale=[3, 0.05, 0.8], pose=SE3(0, 1.5, 0.4), color=[0.5, 0.9, 0.5, 0.5], collision=True)
        fence2 = Cuboid(scale=[3, 0.05, 0.8], pose=SE3(0, -1.5, 0.4), color=[0.5, 0.9, 0.5, 0.5], collision=True)
        fence3 = Cuboid(scale=[0.05, 3, 0.8], pose=SE3(1.5, 0, 0.4), color=[0.5, 0.9, 0.5, 0.5], collision=True)
        fence4 = Cuboid(scale=[0.05, 3, 0.8], pose=SE3(-1.5, 0, 0.4), color=[0.5, 0.9, 0.5, 0.5], collision=True)
        # create an UR3
        robot = UR3()
        robot.q = np.zeros(6) 
        tool_offset = 0.001   # <--- tune this to match actual gripper length
        robot.tool = SE3(0, 0, tool_offset)
        #Import bricks
        current_dir = os.path.dirname(os.path.abspath(__file__))
        stl_path = os.path.join(current_dir, "Brick.stl")

        # create and launch a swift environment
        env = swift.Swift()
        env.launch(realTime=True)
        env.set_camera_pose([1.5, 1.5, 1.5], [0, 0, -pi/4])

        #Add objects and robot to the environment
        robot.add_to_env(env)
        env.add(fence1)
        env.add(fence2)
        env.add(fence3)
        env.add(fence4)
        env.add(ground)
        env.add(rail1)
        env.add(rail2)

        #Import and add bricks
        bricks = []
        brick_offset = 0.025
        for i in range(len(brick_pose)):
            brick = geometry.Mesh(stl_path, pose = brick_pose[i], color = (0.4,0,0,1))
            env.add(brick)
            bricks.append(brick)

        input('Press enter to continue.\n')


                                                                                            # for each brick

        is_collision_check = True
        checked_till_waypoint = 0
        q_matrix = []
        collisions = [] 
        while is_collision_check:                                                                                         # while collision not checked
            start_waypoint = checked_till_waypoint
            for i in range(len(brick_pose)):

                # end effector transform
                T1 = robot.fkine(robot.q)                                                                                       # pose of end effector
                q_robot = robot.ikine_LM(T1, q0 = robot.q).q                                                                    # find joint angles from inverse kinematics
                print(f"Start point: \n{T1}")                                                                                   # print starting pose of end effector
                q_matrix_join = interpolate_waypoints_radians([T1, brick_pose[i]])

                if not is_collision(robot, q_matrix_join, faces, vertices, faces_normal, collisions, return_once_found=True):   # check for collision
                    q_matrix.extend(q_matrix_join)                                                                 # add to trajectory
                    for q in q_matrix_join:                                                                                              # for each set of joint angles in the trajectory
                        robot.q = q                                                                                             # update robot joint angles                                                                                                                    
                        env.step(0.05)                                                                                          # step the environment
                        time.sleep(0.01)                                                                                        # wait a short time to make the animation visible
                    is_collision_check = False
                    checked_till_waypoint = i+1
                    
                    q_matrix_join = interpolate_waypoints_radians([q_matrix[-1], brick_pose[i]])

                    if not is_collision(robot, q_matrix_join, faces, vertices, faces_normal, collisions, return_once_found=True):   # check for collision
                        q_matrix.extend(q_matrix_join)                                                                 # add to trajectory
                        for q in q_matrix_join:                                                                                              # for each set of joint angles in the trajectory
                            robot.q = q                                                                                             # update robot joint angles                                                                                                                    
                            env.step(0.05)                                                                                          # step the environment
                            time.sleep(0.01)                                                                                                         # exit for loop
                        break
                else: 
                    # Randomly pick a pose that is not in collision
                    q_rand = (2 * np.random.rand(1, 3) - 1) * np.pi
                    q_rand = q_rand.tolist()[0]  # Convert to a 3-element list

                    while is_collision(robot, [q_rand], faces, vertices, faces_normal, collisions, return_once_found=True):
                        q_rand = (2 * np.random.rand(1, 3) - 1) * np.pi
                        q_rand = q_rand.tolist()[0]  # Convert to a 3-element list
                    q_waypoints = np.concatenate((q_waypoints[:i+1], [q_rand], q_waypoints[i+1:]), axis=0)
                    is_collision_check = True
                    break    

        # Check again
        if is_collision(robot, q_matrix, faces, vertices, faces_normal, collisions, env=env, return_once_found=False):
            print('Collision detected!')
        else:
            print('No collision found')
                
                # q_brick = robot.ikine_LM(brick_pose[i] * SE3(0, 0, brick_offset), q0 = robot.q).q                                                         # find joint angles from inverse kinematics
                # print (f"Brick: \n{brick_pose[i]}")                                                                             # print pose of brick

                # # confirm joint angles are within limits and generate trajectory
                # q2_joint_limits = check_joint_limits(q_brick, robot)                                                       # check if joint angles are within limits
                # q_matrix = jtraj(q_robot, q_brick, steps).q                                                                 # generate a smooth joint trajectory

                # # animate
                # for q in q_matrix:                                                                                              # for each set of joint angles in the trajectory
                #     if q2_joint_limits==True:                                                                                   # if the joint angles are valid       
                #         robot.q = q                                                                                             # update robot joint angles                                                                                                                    
                #         env.step(0.05)                                                                                          # step the environment
                #         time.sleep(0.02)                                                                                        # wait a short time to make the animation visible   
                #     else:
                #         print("ERROR: Goal joint angles exceed limits - cannot move to goal")                                   #print error message
                #         break                                                                                                                 
                
                # print(f"End point: \n{robot.fkine(robot.q)}")                                                                   # print end pose of end effector         
                # print("------------------------------------------------------------------------------------------------")       # print a separator line
                
                # # end effector transform
                # T1 = robot.fkine(robot.q)                                                                                       # pose of end effector
                # q_robot2 = robot.ikine_LM(T1, q0 = robot.q).q 
                # print(f"Start point: \n{T1}")                                                                                   # print starting pose of end effector

                # # wall transform
                # q_wall = robot.ikine_LM(wall_pose[i], q0 = np.zeros(6)).q                                                       # find joint angles from inverse kinematics                    
                # print (f"Wall: \n{wall_pose[i]}")                                                                               # print pose of wall    

                # # confirm joint angles are within limits abnd generate trajectory
                # q_goal_joint_limits = check_joint_limits(q_wall, robot)                                                    # check if joint angles are within limits    
                # q_matrix = jtraj(q_robot2, q_wall, steps).q                                                                 # generate a smooth joint trajectory  

                # # animate
                # for q in q_matrix:                                                                                              # for each set of joint angles in the trajectory
                #     if q_goal_joint_limits==True:                                                                               # if the joint angles are valid 
                #         robot.q = q                                                                                             # update robot joint angles  
                #         bricks[i].T = robot.fkine(robot.q)                                                                      # update brick position
                #         env.step(0.05)                                                                                          # step the environment
                #         time.sleep(0.02)                                                                                        # wait a short time to make the animation visible
                #     else:
                #         print("Goal joint angles exceed limits - cannot move to goal")                                          # print error message
                #         break

                # #print end robot pose
                # print(f"End point: \n{robot.fkine(robot.q)}")                                                                   # print end pose of end effector  
                # print("------------------------------------------------------------------------------------------------")       # print a separator line

        env.hold()


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

def fine_interpolation(q1, q2, max_step_radians = np.deg2rad(1))->np.ndarray:
    """
    Use results from Q2.6 to keep calling jtraj until all step sizes are
    smaller than a given max steps size
    """
    steps = 2
    while np.any(max_step_radians < np.abs(np.diff(jtraj(q1,q2,steps).q, axis= 0))):
        steps+=1
    return jtraj(q1,q2,steps).q

def interpolate_waypoints_radians(waypoint_radians, max_step_radians = np.deg2rad(1))->np.ndarray:
    """
    Given a set of waypoints, finely intepolate them
    """
    q_matrix = []
    for i in range(np.size(waypoint_radians,0)-1):
        for q in fine_interpolation(waypoint_radians[i], waypoint_radians[i+1], max_step_radians):
            q_matrix.append(q)
    return q_matrix
    
def is_collision(robot, q_matrix, faces, vertex, face_normals, collisions=[], env=None, return_once_found=True):
    """
    This is based upon the output of questions 2.5 and 2.6
    Given a robot model (robot), and trajectory (i.e. joint state vector) (q_matrix)
    and triangle obstacles in the environment (faces,vertex,face_normals)
    """
    result = False
    for i, q in enumerate(q_matrix):
        # Get the transform of every joint (i.e. start and end of every link)
        tr = robot.fkine(robot.q)
        
        # Go through each link and also each triangle face
        for i in range(np.size(tr,2)-1):
            for j, face in enumerate(faces):
                vert_on_plane = vertex[face][0]
                intersect_p, check = line_plane_intersection(face_normals[j], 
                                                            vert_on_plane, 
                                                            tr[i][:3,3], 
                                                            tr[i+1][:3,3])
                # list of all triangle combination in a face
                triangle_list  = np.array(list(combinations(face,3)),dtype= int)
                if check == 1:
                    for triangle in triangle_list:
                        if is_intersection_point_inside_triangle(intersect_p, vertex[triangle]):
                            # Create a red sphere in Swift at the intersection point IF environment passed - if lagging, reduce radius
                            if env is not None:
                                new_collision = Sphere(radius=0.05, color=[1.0, 0.0, 0.0, 1.0])
                                new_collision.T = transl(intersect_p[0], intersect_p[1], intersect_p[2])
                                env.add(new_collision)
                                collisions.append(new_collision)
                            result = True
                            if return_once_found:
                                return result
                            break
    return result
    
def is_intersection_point_inside_triangle(intersect_p, triangle_verts):
    u = triangle_verts[1, :] - triangle_verts[0, :]
    v = triangle_verts[2, :] - triangle_verts[0, :]

    uu = np.dot(u, u)
    uv = np.dot(u, v)
    vv = np.dot(v, v)

    w = intersect_p - triangle_verts[0, :]
    wu = np.dot(w, u)
    wv = np.dot(w, v)

    D = uv * uv - uu * vv

    # Get and test parametric coords (s and t)
    s = (uv * wv - vv * wu) / D
    if s < 0.0 or s > 1.0:  # intersect_p is outside Triangle
        return 0

    t = (uv * wu - uu * wv) / D
    if t < 0.0 or (s + t) > 1.0:  # intersect_p is outside Triangle
        return False

    return True  # intersect_p is in Triangle

            

if __name__ == "__main__":
    LabAT1().do_it()