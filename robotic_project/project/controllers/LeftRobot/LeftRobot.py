from controller import Supervisor
import numpy as np
import math
from collections import deque
import time
import warnings


class RobotController(Supervisor):
    def __init__(self, robot_name):
        super(RobotController, self).__init__()  # Use super() for base class initialization
        self.timestep = int(self.getBasicTimeStep())
        self.emitter = self.getDevice("emitter")
        # self.emitter.enable()
        self.robot_name = robot_name

        # Initialize Wheels
        
        # Initialize Wheels and Arms
        self.front_right_wheel = self.getDevice("wheel1")
        self.front_right_wheel_sensor = self.getDevice("wheel1sensor")
        self.front_right_wheel_sensor.enable(self.timestep)
        self.front_left_wheel = self.getDevice("wheel2")
        self.front_left_wheel_sensor = self.getDevice("wheel2sensor")
        self.front_left_wheel_sensor.enable(self.timestep)
        self.back_right_wheel = self.getDevice("wheel3")
        self.back_right_wheel_sensor = self.getDevice("wheel3sensor")
        self.back_right_wheel_sensor.enable(self.timestep)
        self.back_left_wheel = self.getDevice("wheel4")
        self.back_left_wheel_sensor = self.getDevice("wheel4sensor")
        self.back_left_wheel_sensor.enable(self.timestep)

        self.arm1 = self.getDevice("arm1")
        self.arm2 = self.getDevice("arm2")
        self.arm3 = self.getDevice("arm3")
        self.arm4 = self.getDevice("arm4")
        self.arm5 = self.getDevice("arm5")
        self.left_finger = self.getDevice("finger::left")
        self.right_finger = self.getDevice("finger::right")

        self.arm1.setPosition(float("inf"))
        self.arm2.setPosition(float("inf"))
        self.arm3.setPosition(float("inf"))
        self.arm4.setPosition(float("inf"))
        self.arm5.setPosition(float("inf"))
        self.left_finger.setPosition(float("inf"))
        self.right_finger.setPosition(float("inf"))

        self.arm1.setVelocity(0)
        self.arm2.setVelocity(0)
        self.arm3.setVelocity(0)
        self.arm4.setVelocity(0)
        self.arm5.setVelocity(0)
        self.left_finger.setVelocity(0)
        self.right_finger.setVelocity(0)

        # Set wheels to velocity control
        for wheel in [self.front_right_wheel, self.front_left_wheel, 
                      self.back_right_wheel, self.back_left_wheel]:
            wheel.setPosition(float("inf"))
            wheel.setVelocity(0)

        # Initialize downward-facing camera
        self.downward_camera = self.getDevice("downward_camera")
        self.downward_camera.enable(self.timestep)

        # Initialize forward-facing camera
        # self.forward_camera = self.getDevice("forward_camera")
        # self.forward_camera.enable(self.timestep)

        # Initialize DistanceSensor
        self.distance_sensor = self.getDevice("distance sensor")
        self.distance_sensor.enable(self.timestep)

        # Initialize Backward DistanceSensor
        # self.back_distance_sensor = self.getDevice("backward distance sensor")
        # self.back_distance_sensor.enable(self.timestep)

        # Adding GPS and Compass in the RobotController init method
        self.gps = self.getDevice("gps")
        self.gps.enable(self.timestep)

        self.compass = self.getDevice("compass")
        self.compass.enable(self.timestep)

        # Color detection storage
        self.recognized_colors = []
        self.last_detected_color = None
        self.color_positions_left = {
            'red': {'x': 0.55, 'y': 0.33},
            'blue': {'x': 1.05, 'y': 0.33},
            'green': {'x': -1.05, 'y': 0.33},
            'yellow': {'x': -0.45, 'y': 0.33}
        }

    def set_motors_velocity(self, v1, v2, v3, v4):
        self.front_right_wheel.setVelocity(v1)
        self.front_left_wheel.setVelocity(v2)
        self.back_right_wheel.setVelocity(v3)
        self.back_left_wheel.setVelocity(v4)

    def move_left(self, velocity):
        self.front_right_wheel.setVelocity(velocity)
        self.front_left_wheel.setVelocity(-velocity)
        self.back_left_wheel.setVelocity(velocity)
        self.back_right_wheel.setVelocity(-velocity)

    def move_right(self, velocity):
        self.front_right_wheel.setVelocity(-velocity)
        self.front_left_wheel.setVelocity(velocity)
        self.back_left_wheel.setVelocity(-velocity)
        self.back_right_wheel.setVelocity(velocity)

    def turn_cw(self, velocity):
        self.front_right_wheel.setVelocity(-velocity)
        self.front_left_wheel.setVelocity(velocity)
        self.back_left_wheel.setVelocity(velocity)
        self.back_right_wheel.setVelocity(-velocity)

    def turn_ccw(self, velocity):
        self.front_right_wheel.setVelocity(velocity)
        self.front_left_wheel.setVelocity(-velocity)
        self.back_left_wheel.setVelocity(-velocity)
        self.back_right_wheel.setVelocity(velocity)

    def wheel_value(self):
        wheel4value = self.back_left_wheel_sensor.getValue()
        wheel3value = self.back_right_wheel_sensor.getValue()
        wheel2value = self.front_left_wheel_sensor.getValue()
        wheel1value = self.front_right_wheel_sensor.getValue()
        return wheel1value, wheel2value, wheel3value, wheel4value

    def turn_for_rotations(self, num_rotations, velocity):
        # Start turning
        self.turn_cw(velocity) if velocity > 0 else self.turn_ccw(-velocity) 
        self.step(self.timestep)       
        # Get the initial wheel values
        initial_wheel1, initial_wheel2, initial_wheel3, initial_wheel4 = self.wheel_value()
        print(self.wheel_value())
        print(num_rotations)
        if(velocity>0):
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
                # print(f"Current Wheel Values: {current_wheel1}, {current_wheel2}, {current_wheel3}, {current_wheel4}")
                
                # Check if any wheel has reached or surpassed its target position
                if current_wheel1 <= target_wheel1 or \
                current_wheel2 >= target_wheel2 or \
                current_wheel3 <= target_wheel3 or \
                current_wheel4 >= target_wheel4:
                    break

        if(velocity<0):
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
                # print(f"Current Wheel Values: {current_wheel1}, {current_wheel2}, {current_wheel3}, {current_wheel4}")
                
                # Check if any wheel has reached or surpassed its target position
                if current_wheel1 >= target_wheel1 or \
                current_wheel2 <= target_wheel2 or \
                current_wheel3 >= target_wheel3 or \
                current_wheel4 <= target_wheel4:
                    break

        # print (target_wheel1, target_wheel2, target_wheel3, target_wheel4)
        # Monitor the wheel sensors to stop at the right position
        
        # Stop the wheels after completing the turn
        self.set_motors_velocity(0, 0, 0, 0)

    def detect_color(self,cameraNo):
        image = cameraNo.getImage()
        height = cameraNo.getHeight()
        width = cameraNo.getWidth()
        buffer = np.frombuffer(image, np.uint8).reshape((height, width, 4))

        # Thresholds based on observed values
        # Yellow
        yellow_mask =((buffer[:,:,0] < 25) & 
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
            current_position = self.gps.getValues()
            current_x, _, _ = current_position
            # print(current_x)

            dx = target_x - current_x
            
            # Check if the robot is close enough to the target x position
            if abs(dx) < 0.05:  # threshold distance to consider it reached
                self.set_motors_velocity(0, 0, 0, 0)  # Stop moving
                print("Reached target x position.")
                break
            
            # Determine direction to turn based on sign of dx
            if dx > 0:
                self.move_left(5)  # Adjust the velocity as necessary
            else:
                self.move_right(5)  # Adjust the velocity as necessary

    def move_y(self, target_y):
        # Loop to adjust the robot's forward/backward position
        while self.step(self.timestep) != -1:
            current_position = self.gps.getValues()
            _, current_y, _ = current_position
            # print(current_y)
            
            dy = target_y - current_y
            
            # Check if the robot is close enough to the target z position
            if abs(dy) < 0.05:  # threshold distance to consider it reached
                self.set_motors_velocity(0, 0, 0, 0)  # Stop moving
                print("Reached target y (z) position.")
                break
            
            # Move forward or backward based on the sign of dy
            if dy < 0:
                self.set_motors_velocity(5, 5, 5, 5)  # Move forward
            else:
                self.set_motors_velocity(-5, -5, -5, -5)  # Move backward

    def grip_box(self):

        # Position the arm joints
        self.arm2.setPosition(-1.6)
        self.arm3.setPosition(-1.05)
        self.arm4.setPosition(-0.9)
        
        # Set velocities for the arm movements
        self.arm2.setVelocity(1)
        self.arm3.setVelocity(1)
        self.arm4.setVelocity(1)

        # Allow time for arms to move into position
        for _ in range(50):
            if self.step(self.timestep) == -1:
                break

        # Move fingers to grip position
        self.left_finger.setPosition(0.05)
        self.right_finger.setPosition(0.05)
        self.left_finger.setVelocity(1)
        self.right_finger.setVelocity(1)

        # Wait for fingers to close around the box
        for _ in range(20):
            if self.step(self.timestep) == -1:
                break

        # Set final grip positions
        self.left_finger.setPosition(-0.06)
        self.right_finger.setPosition(-0.06)
        self.arm2.setPosition(0)  # Optionally reset the first joint

        # Final adjustment phase
        for _ in range(20):
            if self.step(self.timestep) == -1:
                break

    def rotate_arm1(self):
        # Fetch the arm1 rotational motor
        arm1 = self.getDevice("arm1")
        
        # Set the target position for a 180-degree rotation (π radians)
        arm1.setPosition(-2.9496)  # 180 degrees in radians
        
        # Set a reasonable velocity for smooth movement
        arm1.setVelocity(1)  # Adjust the speed as needed
        
        # Wait for the rotation to complete
        for _ in range(100):  # Adjust the range for sufficient movement time
            if self.step(self.timestep) == -1:
                break

    def release_box(self):

        # Open fingers to release the box
        self.arm2.setPosition(-0.43)
        self.arm3.setPosition(-1.55)
        self.arm4.setPosition(-0.8)
        self.arm2.setVelocity(1)
        self.arm4.setVelocity(1)
        for _ in range(100):
            if self.step(self.timestep) == -1:
                break
        
        self.left_finger.setPosition(0.06)  # Open finger position
        self.right_finger.setPosition(0.06)
        self.left_finger.setVelocity(1)
        self.right_finger.setVelocity(1)

        # Wait for fingers to open
        for _ in range(30):
            if self.step(self.timestep) == -1:
                break
        

        # Reset fingers to a fully open position if necessary
        
        
        self.rotate_arm1()
        # Reset arm joints to 'home' or 'neutral' positions
        self.arm1.setPosition(0)  # Example home position for arm1
        self.arm2.setPosition(0)  # Example home position for arm2
        self.arm3.setPosition(0)  # Example home position for arm3
        self.arm4.setPosition(0)  # Example home position for arm4
        self.arm1.setVelocity(1)
        self.arm2.setVelocity(1)
        self.arm3.setVelocity(1)
        self.arm4.setVelocity(1)
        self.turn_for_rotations(1,-1)
        for _ in range(20):
            if self.step(self.timestep) == -1:
                break
        self.left_finger.setPosition(0)
        self.right_finger.setPosition(0)
        self.left_finger.setVelocity(1)
        self.right_finger.setVelocity(1)
            
        # arm4.setPosition(-0.825)

    def hold_boxes(self, color):
        self.grip_box()
        self.turn_for_rotations(1,1)
        self.rotate_arm1()
        for _ in range(50):
            if self.step(self.timestep) == -1:
                break
        self.release_box()
        for _ in range(50):
            if self.step(self.timestep) == -1:
                break
        # print("sending color message")
        # self.emitter.send(color.encode('utf-8'))
        # print("message sent")

    def send_message(self, message):
        print("Sending 'done' message to the right robot...")
        self.emitter.send("done".encode('utf-8'))
        print("Message sent")

    def complete_tasks(self):
        print("Starting main loop...")
        self.set_motors_velocity(10, 10, 10, 10)  # Start by moving forward or at a default pace

        while self.step(self.timestep) != -1:
            current_color = self.detect_color(self.downward_camera)
            distance = self.distance_sensor.getValue()
            # back_distance = self.back_distance_sensor.getValue()
            # back_color = self.detect_color(self.forward_camera)
            # print("Front Distance:", distance, "Back Distance:", back_distance)

            # Display current seeing color if it changes from the last detected color
            if current_color != self.last_detected_color:
                print(f"Currently seeing: {current_color}")
            self.last_detected_color = current_color

            # Check if new colors are detected and add them to the list
            if current_color != 'none' and (current_color not in self.recognized_colors) and distance >= 50:
                self.recognized_colors.append(current_color)
                print(f"New color detected and added: {current_color} with {self.robot_name}")

            # Execute movement based on recognized colors
            while self.recognized_colors and distance <= 30:
                # print("Sending completion signal...")
                # self.emitter.send("done")
                # print("Signal sent.")
                self.set_motors_velocity(0, 0, 0, 0)
                color_info = self.recognized_colors[0] 
                print('going to: ', color_info)
                target_x = self.color_positions_left[color_info]['x']
                target_y = self.color_positions_left[color_info]['y']
                print(color_info)
                self.move_x(target_x)
                self.move_y(target_y)
                self.hold_boxes(color_info)
                print(f"Reached position for {color_info}")
                self.recognized_colors.pop(0)
                print(self.recognized_colors)
                            
            if not self.recognized_colors and distance <= 40:
                print('end')
                # print('Moving default to detect colors...')
                self.set_motors_velocity(0, 0, 0, 0)  # Continue moving to detect colors
                break

        print("Final loop has ended.")
        pass
        
    def run(self):
        self.step(self.timestep) 
        # Robot 1 completes its tasks
        self.complete_tasks()
        # Signal completion
        print("Sending completion signal...")
        self.emitter.send("done")
        print("Signal sent.")

        

    # def run(self):
        # while self.step(self.timestep) != -1:
        #     # It's better to call getFromDef inside a method that's regularly called
        #     red_box_node = self.getFromDef("RED_BOX")  # Ensure the DEF name is correct

        #     if red_box_node is not None:
        #         position = red_box_node.getField("translation").getSFVec3f()
        #         print(f"KUKA Box Position: {position[0]}, {position[1]}, {position[2]}")
        #     else:
        #         print("KUKA box node not found")
        #         break  # Exit loop if node not found

if __name__ == "__main__":
    robot_controller = RobotController("youBotLeft")
    warnings.filterwarnings('ignore')
    robot_controller.run()