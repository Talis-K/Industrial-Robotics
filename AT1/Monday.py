#!/usr/bin/env python3
import time
import roslibpy
import numpy as np

current_pos = None

def call_service(client, service_name):
    """Call a std_srvs/Trigger service and print the response."""
    service = roslibpy.Service(client, service_name, 'std_srvs/Trigger')
    request = roslibpy.ServiceRequest({})  # Trigger service takes no arguments

    print(f"[ROS] Calling service: {service_name}")
    result = service.call(request)
    print(f"[ROS] Response: success={result['success']}, message='{result['message']}'")

def joint_state_cb(message):
    global current_pos
    # The JointState message contains 'position' array
    current_pos = list(message['position'])

def move_ur_joint_positions(client,joint_positions, duration=5.0):
    global current_pos
    #client = roslibpy.Ros(host='192.168.27.1', port=9090)  # Replace with your ROS bridge IP

    try:
        client = roslibpy.Ros(host='192.168.27.1', port=9090)  # Replace with your ROS bridge IP

        client.run()

        # Subscribe to joint states to get the current position
        listener = roslibpy.Topic(client, '/ur/joint_states', 'sensor_msgs/JointState')
        listener.subscribe(joint_state_cb)

        # Wait until we receive a joint state
        print("[ROS] Waiting for current joint state...")
        start_time = time.time()
        while current_pos is None and time.time() - start_time < 5.0:
            time.sleep(0.05)
        if current_pos is None:
            raise RuntimeError("No joint state received from /ur/joint_states")

        print(f"[ROS] Current joint positions: {current_pos}")

        # Build a JointTrajectory message for the scaled_pos_joint_traj_controller
        joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        trajectory_msg = {
            'joint_names': joint_names,
            'points': [
                {
                    'positions': current_pos,
                    'time_from_start': {'secs': 0, 'nsecs': 0}
                },
                {
                    'positions': joint_positions,
                    'time_from_start': {
                        'secs': int(duration),
                        'nsecs': int((duration - int(duration)) * 1e9)
                    }
                }
            ]
        }

        # Publish to the controller's /command topic
        topic = roslibpy.Topic(
            client,
            '/ur/scaled_pos_joint_traj_controller/command',
            'trajectory_msgs/JointTrajectory'
        )
        topic.advertise()
        topic.publish(roslibpy.Message(trajectory_msg))
        print("[ROS] Trajectory published.")

        # Wait for motion to complete
        time.sleep(duration + 1.0)

        topic.unadvertise()
        listener.unsubscribe()

    finally:
        pass
        ##client.terminate()
        #print("[ROS] Disconnected from rosbridge.")

if __name__ == '__main__':
    try:
        client = roslibpy.Ros(host='192.168.27.1', port=9090)  # Replace with your ROS bridge IP

        client.run()
        # Example: move slightly and back
        ##target_joint_positions = [0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0]
        ##move_ur_joint_positions(client, target_joint_positions, duration=7.0)

        # Open gripper
        
        target_joint_positions = [0.63, -1.32, 1.2, -1.47, -1.54, 0.002]
        move_ur_joint_positions(client,target_joint_positions, duration=7.0)

        call_service(client, '/onrobot/open')
        time.sleep(2)

        # Close gripper
        call_service(client, '/onrobot/close')
        time.sleep(2)


        target_joint_positions = [1.7473816871643066, -0.9613179129413147, 0.8758967558490198, -1.5557692807963868, -1.5483344236956995, 0.009369985200464725]
       
        move_ur_joint_positions(client,target_joint_positions, duration=7.0)

         # Open gripper  
        call_service(client, '/onrobot/open')
        time.sleep(2)

        # Close gripper
        call_service(client, '/onrobot/close')
        time.sleep(2)

        # Open gripper  
        call_service(client, '/onrobot/open')
        time.sleep(2)


        

        
    except KeyboardInterrupt:
        print("\n[APP] Interrupted by user.")
