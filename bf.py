from simple_pid import PID
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

import numpy as np
import time

import numpy as np
import time
from simple_pid import PID

class MotionTracker:
    def __init__(self, vehicle, dt):
        self.vehicle = vehicle
        self.dt = dt
        self.position = np.zeros(3)  # [x, y, z]
        self.velocity = np.zeros(3)
        self.orientation = np.zeros(3)  # [roll, pitch, yaw]
        self.last_update = time.time()

    def update(self):
        current_time = time.time()
        dt = current_time - self.last_update
        self.last_update = current_time
        self.velocity = np.array(self.vehicle.velocity)
        self.orientation = np.array([self.vehicle.attitude.roll,
                                     self.vehicle.attitude.pitch,
                                     self.vehicle.attitude.yaw])
        self.position += self.velocity * dt

    def reset(self):
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = np.zeros(3)

    def get_position(self):
        return self.position

    def get_velocity(self):
        return self.velocity



class DroneController:
    def __init__(self, MotionTracker,vehicle):
        self.vehicle = vehicle
        self.MotionTracker = MotionTracker
        self.pid_x = PID(0.8, 0.05, 0.02, setpoint=0)
        self.pid_y = PID(0.8, 0.05, 0.02, setpoint=0)
        self.pid_z = PID(0.8, 0.05, 0.02, setpoint=0)
        self.pid_yaw = PID(0.5, 0.05, 0.02, setpoint=0)
        self.pid_yaw.output_limits = (-0.05, 0.05)  # Reduced yaw rate for smoother rotation
        self.pid_x.output_limits = (-1.0, 1.0)
        self.pid_y.output_limits = (-1.0, 1.0)
        self.pid_z.output_limits = (-1.0, 1.0)


    def arm_and_takeoff(self,target_altitude):
        print("Arming motors...")
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to become armable...")
            time.sleep(1)

        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(target_altitude)

        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    def send_velocity(self, vx, vy, vz, yaw_rate=0):
        # Use MAV_FRAME_LOCAL_NED to ensure movement in the global frame
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # Bitmask to control velocity (vx, vy, vz) and yaw_rate
            0, 0, 0,  # No need to set position
            vx, vy, vz,  # Desired velocity in NED frame
            0, 0, 0,  # No acceleration control
            yaw_rate, 0  # Yaw rate control
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def move_forward(self, distance):
        self.MotionTracker.reset()
        target_dist = self.MotionTracker.position[0]+distance
        while abs(target_dist-self.MotionTracker.position[0])>0.05:
            self.MotionTracker.update()
            print(self.MotionTracker.position)
            velocity_x = self.pid_x(target_dist - self.MotionTracker.position[0])
            self.send_velocity(velocity_x, 0, 0)
            time.sleep(1)

        print("forwarded")

    def move_backward(self, distance ):
        self.MotionTracker.reset()
        target_dist = self.MotionTracker.position[0] - distance
        while (-target_dist + self.MotionTracker.position[0]) > 0.05:
            self.MotionTracker.update()
            velocity_x = self.pid_x(-target_dist + self.MotionTracker.position[0])
            self.send_velocity(-velocity_x, 0, 0)
            time.sleep(0.1)
            print("backed")


    def move_left(self, distance):
        self.MotionTracker.reset()
        target_dist = self.MotionTracker.position[1] - distance
        while (-target_dist + self.MotionTracker.position[1]) > 0.05:
            self.MotionTracker.update()
            velocity_y = self.pid_y(-target_dist + self.MotionTracker.position[1])
            self.send_velocity(0,-velocity_y, 0)
            time.sleep(0.1)
            print("moved left")

    def move_right(self, distance):
        curr_distance = 0
        velocity_y = 1  # Set velocity in the Y direction (East)

        while abs(distance - curr_distance) > 0.05:
            # Update current distance traveled in the Y direction (East)
            curr_distance += self.vehicle.velocity[1] * 0.1  # Update using the current velocity and time step

            # Send velocity in the East direction (Y axis in NED frame)
            self.send_velocity(0, velocity_y, 0)

            # Wait for 100ms before the next update
            time.sleep(0.1)

        # Stop movement after reaching the target distance
        self.send_velocity(0, 0, 0)
        print("moved right")

    def move_up(self, altitude):
        self.MotionTracker.reset()
        target_dist = self.MotionTracker.position[2] - altitude
        while (-target_dist + self.MotionTracker.position[2]) > 0.05:
            self.MotionTracker.update()
            velocity_z = self.pid_z(-target_dist + self.MotionTracker.position[2])
            self.send_velocity(0, 0,-velocity_z)
            time.sleep(0.1)
            print("moved up")


    def move_down(self, altitude):
        self.MotionTracker.reset()
        target_dist = self.MotionTracker.position[2] + altitude
        while (target_dist - self.MotionTracker.position[2]) > 0.05:
            self.MotionTracker.update()
            velocity_z = self.pid_z(target_dist - self.MotionTracker.position[2])
            self.send_velocity(0, 0, velocity_z)
            time.sleep(0.1)
            print("moved down")

    def rotate_yaw(self, yaw):
        self.MotionTracker.reset()
        target_yaw = self.MotionTracker.orientation[2] + yaw
        self.pid_yaw.setpoint = target_yaw
        while abs(self.MotionTracker.orientation[2] - target_yaw) > 0.05:
            self.MotionTracker.update()
            r = self.pid_yaw(self.MotionTracker.orientation[2])
            self.send_velocity(0, 0, 0, r)
            time.sleep(0.1)
            print("yawed")

    def stop(self):
        self.send_velocity(0, 0, 0, 0)
        print("stopped")

vehicle = connect('127.0.0.1:14550', wait_ready=True)
# print(dir(vehicle))
# # #
# print(vehicle.velocity)

motion_tracker = MotionTracker(vehicle,0.1)

time.sleep(5)  # Delay to stabilize the IMU

print("Bias calibrated.")

controller = DroneController(motion_tracker,vehicle)
# controller.arm_and_takeoff(10)
#controller.move_forward(2)
controller.move_right(20)
# controller.move_backward(2)
# controller.move_left(2)

time.sleep(0.1)

controller.stop()
