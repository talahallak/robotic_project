from controller import Supervisor
import numpy as np
import math
import warnings
from collections import deque

warnings.filterwarnings('ignore')


class RobotController(Supervisor):
    def __init__(self, robot_name):
        super(RobotController, self).__init__()  # Use super() for base class initialization
        self.timestep = int(self.getBasicTimeStep())
        self.receiver = self.getDevice("receiver")
        self.receiver.enable(self.timestep)
        self.robot_name = robot_name

        # Initialize Wheels and Sensors
        self.front_right_wheel_RR = self.getDevice("wheel1")
        self.front_right_wheel_sensor_RR = self.getDevice("wheel1sensor")
        self.front_right_wheel_sensor_RR.enable(self.timestep)
        self.front_left_wheel_RR = self.getDevice("wheel2")
        self.front_left_wheel_sensor_RR = self.getDevice("wheel2sensor")
        self.front_left_wheel_sensor_RR.enable(self.timestep)
        self.back_right_wheel_RR = self.getDevice("wheel3")
        self.back_right_wheel_sensor_RR = self.getDevice("wheel3sensor")
        self.back_right_wheel_sensor_RR.enable(self.timestep)
        self.back_left_wheel_RR = self.getDevice("wheel4")
        self.back_left_wheel_sensor_RR = self.getDevice("wheel4sensor")
        self.back_left_wheel_sensor_RR.enable(self.timestep)

        # Initialize Arms and Fingers
        self.arm1_RR = self.getDevice("arm1")
        self.arm2_RR = self.getDevice("arm2")
        self.arm3_RR = self.getDevice("arm3")
        self.arm4_RR = self.getDevice("arm4")
        self.arm5_RR = self.getDevice("arm5")
        self.left_finger_RR = self.getDevice("finger::left")
        self.right_finger_RR = self.getDevice("finger::right")

        # Set arms and fingers to velocity control
        for arm in [self.arm1_RR, self.arm2_RR, self.arm3_RR, self.arm4_RR, self.arm5_RR, self.left_finger_RR, self.right_finger_RR]:
            arm.setPosition(float("inf"))
            arm.setVelocity(0)

        # Set wheels to velocity control
        for wheel in [self.front_right_wheel_RR, self.front_left_wheel_RR, 
                      self.back_right_wheel_RR, self.back_left_wheel_RR]:
            wheel.setPosition(float("inf"))
            wheel.setVelocity(0)

        # Initialize Cameras
        self.downward_camera_RR = self.getDevice("downward_camera")
        self.downward_camera_RR.enable(self.timestep)

        # self.forward_camera = self.getDevice("forward_camera")
        # self.forward_camera.enable(self.timestep)

        # Initialize Distance Sensors
        self.distance_sensor_RR = self.getDevice("distance sensor")
        self.distance_sensor_RR.enable(self.timestep)

        # Initialize GPS and Compass
        self.gps_RR = self.getDevice("gps")
        self.gps_RR.enable(self.timestep)

        self.compass_RR = self.getDevice("compass")
        self.compass_RR.enable(self.timestep)

        # Initialize color_positions_left as empty dict; it will be populated dynamically
        self.color_positions_left = {}

        # Color detection storage
        self.recognized_colors = []
        self.last_detected_color = None

    def set_motors_velocity_RR(self, v1, v2, v3, v4):
        self.front_right_wheel_RR.setVelocity(v1)
        self.front_left_wheel_RR.setVelocity(v2)
        self.back_right_wheel_RR.setVelocity(v3)
        self.back_left_wheel_RR.setVelocity(v4)

    def move_left(self, velocity):
        self.front_right_wheel_RR.setVelocity(velocity)
        self.front_left_wheel_RR.setVelocity(-velocity)
        self.back_left_wheel_RR.setVelocity(velocity)
        self.back_right_wheel_RR.setVelocity(-velocity)

    def move_right(self, velocity):
        self.front_right_wheel_RR.setVelocity(-velocity)
        self.front_left_wheel_RR.setVelocity(velocity)
        self.back_left_wheel_RR.setVelocity(-velocity)
        self.back_right_wheel_RR.setVelocity(velocity)

    def turn_cw(self, velocity):
        self.front_right_wheel_RR.setVelocity(-velocity)
        self.front_left_wheel_RR.setVelocity(velocity)
        self.back_left_wheel_RR.setVelocity(velocity)
        self.back_right_wheel_RR.setVelocity(-velocity)

    def turn_ccw(self, velocity):
        self.front_right_wheel_RR.setVelocity(velocity)
        self.front_left_wheel_RR.setVelocity(-velocity)
        self.back_left_wheel_RR.setVelocity(-velocity)
        self.back_right_wheel_RR.setVelocity(velocity)

    def wheel_value(self):
        wheel4value = self.back_left_wheel_sensor_RR.getValue()
        wheel3value = self.back_right_wheel_sensor_RR.getValue()
        wheel2value = self.front_left_wheel_sensor_RR.getValue()
        wheel1value = self.front_right_wheel_sensor_RR.getValue()
        return wheel1value, wheel2value, wheel3value, wheel4value

    def turn_for_rotations(self, num_rotations, velocity):
        # Start turning
        if velocity > 0:
            self.turn_cw(velocity)
        else:
            self.turn_ccw(-velocity)
        self.step(self.timestep)       
        # Get the initial wheel values
        initial_wheel1, initial_wheel2, initial_wheel3, initial_wheel4 = self.wheel_value()
        print(self.wheel_value())
        print(num_rotations)
        if velocity > 0:
            target_wheel1 = initial_wheel1 - num_rotations  # 2*pi radians per rotation
            print(initial_wheel1, target_wheel1)
            target_wheel2 = initial_wheel2 + num_rotations
            print(initial_wheel2, target_wheel2)
            target_wheel3 = initial_wheel3 - num_rotations
            print(initial_wheel3, target_wheel3)
            target_wheel4 = initial_wheel4 + num_rotations
            print(initial_wheel4, target_wheel4)

            while self.step(self.timestep) != -1:
                current_wheel1, current_wheel2, current_wheel3, current_wheel4 = self.wheel_value()
                # Check if any wheel has reached or surpassed its target position
                if current_wheel1 <= target_wheel1 or \
                   current_wheel2 >= target_wheel2 or \
                   current_wheel3 <= target_wheel3 or \
                   current_wheel4 >= target_wheel4:
                    break

        elif velocity < 0:
            target_wheel1 = initial_wheel1 + num_rotations  # 2*pi radians per rotation
            print(initial_wheel1, target_wheel1)
            target_wheel2 = initial_wheel2 - num_rotations
            print(initial_wheel2, target_wheel2)
            target_wheel3 = initial_wheel3 + num_rotations
            print(initial_wheel3, target_wheel3)
            target_wheel4 = initial_wheel4 - num_rotations
            print(initial_wheel4, target_wheel4)

            while self.step(self.timestep) != -1:
                current_wheel1, current_wheel2, current_wheel3, current_wheel4 = self.wheel_value()
                # Check if any wheel has reached or surpassed its target position
                if current_wheel1 >= target_wheel1 or \
                   current_wheel2 <= target_wheel2 or \
                   current_wheel3 >= target_wheel3 or \
                   current_wheel4 <= target_wheel4:
                    break

        # Stop the wheels after completing the turn
        self.set_motors_velocity_RR(0, 0, 0, 0)

    def detect_color(self, cameraNo):
        image = cameraNo.getImage()
        height = cameraNo.getHeight()
        width = cameraNo.getWidth()
        buffer = np.frombuffer(image, np.uint8).reshape((height, width, 4))

        # Thresholds based on observed values
        # Yellow
        yellow_mask = ((buffer[:,:,0] < 25) & 
                       (buffer[:,:,1] > 50) & (buffer[:,:,1] < 70) & 
                       (buffer[:,:,2] > 40) & (buffer[:,:,2] < 60))
        if np.any(yellow_mask):
            return 'yellow'
        
        # Red
        red_mask = ((buffer[:,:,0] < 20) &
                    (buffer[:,:,1] < 20) & 
                    (buffer[:,:,2] > 70))
        if np.any(red_mask):
            return 'red'
        
        # Blue
        blue_mask = ((buffer[:,:,0] > 65) & (buffer[:,:,0] < 75) &
                    (buffer[:,:,1] < 20) & 
                    (buffer[:,:,2] < 20))
        if np.any(blue_mask):
            return 'blue'
        
        # Green
        green_mask = ((buffer[:,:,0] < 25) &
                      (buffer[:,:,1] > 50) & (buffer[:,:,1] < 60) &
                      (buffer[:,:,2] < 15))
        if np.any(green_mask):
            return 'green'

        return 'none'  # Default if no significant color is detected

    def move_x(self, target_x):
        # Loop to adjust the robot's horizontal position
        while self.step(self.timestep) != -1:
            current_position = self.gps_RR.getValues()
            current_x, _, _ = current_position
            # print(current_x)

            dx = target_x - current_x
            
            # Check if the robot is close enough to the target x position
            if abs(dx) < 0.05:  # threshold distance to consider it reached
                self.set_motors_velocity_RR(0, 0, 0, 0)  # Stop moving
                print("Reached target x position.")
                break
            
            # Determine direction to turn based on sign of dx
            if dx < 0:
                self.move_left(5)  # Adjust the velocity as necessary
            else:
                self.move_right(5)  # Adjust the velocity as necessary

    def move_y(self, target_y):
        # Loop to adjust the robot's forward/backward position
        while self.step(self.timestep) != -1:
            current_position = self.gps_RR.getValues()
            _, current_y, _ = current_position
            # print(current_y)
            
            dy = target_y - current_y
            
            # Check if the robot is close enough to the target y position
            if abs(dy) < 0.05:  # threshold distance to consider it reached
                self.set_motors_velocity_RR(0, 0, 0, 0)  # Stop moving
                print("Reached target y (z) position.")
                break
            
            # Move forward or backward based on the sign of dy
            if dy > 0:
                self.set_motors_velocity_RR(5, 5, 5, 5)  # Move forward
            else:
                self.set_motors_velocity_RR(-5, -5, -5, -5)  # Move backward

    def grip_box(self):
        # Position the arm joints
        self.arm2_RR.setPosition(-1.3)
        self.arm3_RR.setPosition(-1.1)
        self.arm4_RR.setPosition(-0.9)
        
        # Set velocities for the arm movements
        self.arm2_RR.setVelocity(1)
        self.arm3_RR.setVelocity(1)
        self.arm4_RR.setVelocity(1)

        # Allow time for arms to move into position
        for _ in range(50):
            if self.step(self.timestep) == -1:
                break

        # Move fingers to grip position
        self.left_finger_RR.setPosition(0.06)
        self.right_finger_RR.setPosition(0.06)
        self.left_finger_RR.setVelocity(1)
        self.right_finger_RR.setVelocity(1)

        # Wait for fingers to close around the box
        for _ in range(20):
            if self.step(self.timestep) == -1:
                break

        # Set final grip positions
        self.left_finger_RR.setPosition(-0.05)
        self.right_finger_RR.setPosition(-0.05)
        self.arm2_RR.setPosition(0)  # Optionally reset the first joint

        # Final adjustment phase
        for _ in range(20):
            if self.step(self.timestep) == -1:
                break

    def rotate_arm1(self):
        # Fetch the arm1 rotational motor
        # arm1_RR = self.getDevice("arm1")
        
        # Set the target position for a 180-degree rotation (π radians)
        self.arm1_RR.setPosition(-2.9496)  # 180 degrees in radians
        
        # Set a reasonable velocity for smooth movement
        self.arm1_RR.setVelocity(1)  # Adjust the speed as needed
        
        # Wait for the rotation to complete
        for _ in range(100):  # Adjust the range for sufficient movement time
            if self.step(self.timestep) == -1:
                break

    def release_box(self):
        # Open fingers to release the box
        for _ in range(10):
            if self.step(self.timestep) == -1:
                break
        
        self.arm2_RR.setPosition(-1.5)
        self.arm4_RR.setPosition(-1)
        self.arm2_RR.setVelocity(1)
        self.arm4_RR.setVelocity(1)

        for _ in range(30):
            if self.step(self.timestep) == -1:
                break
        
        self.left_finger_RR.setPosition(0.06)  # Open finger position
        self.right_finger_RR.setPosition(0.06)
        self.left_finger_RR.setVelocity(1)
        self.right_finger_RR.setVelocity(1)

        # Wait for fingers to open
        for _ in range(20):
            if self.step(self.timestep) == -1:
                break
        
        self.rotate_arm1()
        
        self.turn_for_rotations(1,-1)

        # Reset fingers to a fully open position if necessary
        self.left_finger_RR.setPosition(0)
        self.right_finger_RR.setPosition(0)
        self.left_finger_RR.setVelocity(1)
        self.right_finger_RR.setVelocity(1)
        # Reset arm joints to 'home' or 'neutral' positions
        self.arm1_RR.setPosition(0)  # Example home position for arm1
        self.arm2_RR.setPosition(0)  # Example home position for arm2
        self.arm3_RR.setPosition(0)  # Example home position for arm3
        self.arm4_RR.setPosition(0)  # Example home position for arm4
        self.arm1_RR.setVelocity(1)
        self.arm2_RR.setVelocity(1)
        self.arm3_RR.setVelocity(1)
        self.arm4_RR.setVelocity(1)
        for _ in range(20):
            if self.step(self.timestep) == -1:
                break

    def hold_boxes(self):
        self.grip_box()
        self.turn_for_rotations(1,1)
        self.rotate_arm1()
        for _ in range(100):
            if self.step(self.timestep) == -1:
                break
        self.release_box()

    def start_operations(self):
        print("Starting main loop...")
        self.set_motors_velocity_RR(10, 10, 10, 10)  # Start by moving forward or at a default pace
        while self.step(self.timestep) != -1:
            current_color = self.detect_color(self.downward_camera_RR)
            distance = self.distance_sensor_RR.getValue()
            # back_distance = self.back_distance_sensor.getValue()
            # back_color = self.detect_color(self.forward_camera)
            # print("Front Distance:", distance, "Back Distance:", back_distance)
            # Display current seeing color if it changes from the last detected color
            if current_color != self.last_detected_color:
                print(f"Currently seeing: {current_color}")
            self.last_detected_color = current_color
            # Check if new colors are detected and add them to the list
            if (current_color != 'none' and 
                (current_color not in self.recognized_colors) and 
                distance >= 50):
                self.recognized_colors.append(current_color)
                print(f"New color detected and added: {current_color} with {self.robot_name}")
            while self.recognized_colors and distance <= 30:
                self.set_motors_velocity_RR(0, 0, 0, 0)
                color_info = self.recognized_colors[0] 
                print('going to: ', color_info)
                # color_received = False
                # while not color_received:
                #     if self.receiver.getQueueLength() > 0:  # Check if a new message has been received
                #         message = self.receiver.getData().decode('utf-8')  # Decode the message
                #         self.receiver.nextPacket()  # Move to the next message
                #         print(f"Received message: {message}")
                        
                #         if message == color_info:  # If the color matches, proceed to move
                #             print(f"Received confirmation for {color_info}, proceeding to move.")
                #             color_received = True
                #         else:
                #             print(f"Waiting for the correct color message... Received: {message}")
                # Now that we've received confirmation, proceed with moving the box
                # error = 0.05 if color_info != 'yellow' and color_info != 'red' else 0.08
                error = -0.03 if color_info == 'red' else 0.05
                # Dynamically get node based on color_info
                color_node = self.getFromDef(color_info.upper() + "_BOX")
                if color_node is not None:
                    position = color_node.getField("translation").getSFVec3f()
                    target_x = position[0] + error if position[0] > 0 else position[0] - error
                    target_y = position[1] - 0.42
                    print(f"Position for {color_info}: {position[0]}, {position[1]}, {position[2]}")
                    print('going to', "y:", target_y)
                    print('going to', "x:", target_x)
                    # Assume move_x and move_y functions accept coordinates directly
                    self.move_x(target_x)
                    self.move_y(target_y)
                    self.hold_boxes()
                    print(f"Reached position for {color_info}")
                else:
                    print(f"{color_info} box node not found")
                self.recognized_colors.pop(0)
                print(f"Remaining recognized colors: {self.recognized_colors}")
                            
            if not self.recognized_colors and distance <= 40:
                # print('Moving default to detect colors...')
                self.set_motors_velocity_RR(0, 0, 0, 0)  # Continue moving to detect colors
        print("Final loop has ended.")
        pass
    
    def run(self):
        while self.step(self.timestep) != -1:
            if self.receiver.getQueueLength() > 0:
                message = self.receiver.getData()
                self.receiver.nextPacket()
                if message == "done":
                    print("Received 'done' message, starting operations...")
                    self.start_operations()
                    break



        
          
    # def start_operations(self):
    #     print("Starting main loop...")
    #     self.set_motors_velocity_RR(10, 10, 10, 10)  # Start by moving forward or at a default pace

    #     while self.step(self.timestep) != -1:
    #         current_color = self.detect_color(self.downward_camera_RR)
    #         distance = self.distance_sensor_RR.getValue()

    #         if current_color != self.last_detected_color:
    #             print(f"Currently seeing: {current_color}")
    #         self.last_detected_color = current_color

    #         # Handle color detection and task execution
    #         if (current_color != 'none' and 
    #             (current_color not in self.recognized_colors) and 
    #             distance >= 50):
    #             self.recognized_colors.append(current_color)
    #             print(f"New color detected and added: {current_color} with {self.robot_name}")

    #         # Process recognized colors
    #         if self.recognized_colors and distance <= 30:
    #             self.process_recognized_colors()

    # def process_recognized_colors(self):
    #     for color_info in self.recognized_colors:
    #         print('Processing color:', color_info)
    #         self.set_motors_velocity_RR(0, 0, 0, 0)  # Stop to handle tasks

    #         if self.check_for_color_message(color_info):
    #             target_x, target_y = self.calculate_target_position(color_info)
    #             self.move_x(target_x)
    #             self.move_y(target_y)
    #             self.hold_boxes()
    #             print(f"Reached position for {color_info}")
    #             self.recognized_colors.remove(color_info)

    # def check_for_color_message(self, color_info):
    #     while self.receiver.getQueueLength() > 0:
    #         message = self.receiver.getData().decode('utf-8')
    #         self.receiver.nextPacket()
    #         print(f"Received message: {message}")

    #         if message == color_info:
    #             print(f"Received confirmation for {color_info}, proceeding to move.")
    #             return True
    #         else:
    #             print(f"Waiting for the correct color message... Received: {message}")
    #     return False

    def run(self):
        while self.step(self.timestep) != -1:
            if self.receiver.getQueueLength() > 0:
                message = self.receiver.getData()
                self.receiver.nextPacket()
                if message == "done":
                    print("Received 'done' message, starting operations...")
                    self.start_operations()
                    break



if __name__ == "__main__":
    robot_controller = RobotController("youBotRight")
    robot_controller.run()
