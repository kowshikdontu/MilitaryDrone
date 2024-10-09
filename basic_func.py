from simple_pid import PID
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

import numpy as np
import time


class MotionTracker:
    def __init__(self, dt, imu):
        self.dt = dt
        self.imu = imu
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.acceleration = np.zeros(3)
        self.orientation = np.zeros(3)  # [roll, pitch, yaw] in radians
        self.bias = np.zeros(3)
        self.last_update = 0
        self.alpha=0.95

    def update(self):
        current_time = time.time()
        self.dt = current_time - self.last_update
        self.last_update = current_time

        accel = self.imu.get_acceleration()
        gyro = self.imu.get_angular_velocity()

        # Estimate pitch and roll from accelerometer data
        accel_pitch = np.arctan2(accel[1], np.sqrt(accel[0] ** 2 + accel[2] ** 2))
        accel_roll = np.arctan2(-accel[0], accel[2])

        # Integrate gyro data
        self.orientation[0] += gyro[0] * self.dt  # Roll
        self.orientation[1] += gyro[1] * self.dt  # Pitch
        self.orientation[2] += gyro[2] * self.dt  # Yaw

        # Complementary filter to combine accelerometer and gyroscope data
        self.orientation[0] = self.alpha * self.orientation[0] + (1 - self.alpha) * accel_roll
        self.orientation[1] = self.alpha * self.orientation[1] + (1 - self.alpha) * accel_pitch

        # Update velocity and position
        self.velocity += accel * self.dt
        self.position += self.velocity * self.dt


    def reset(self):
        # Reset the position, velocity, and orientation
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = np.zeros(3)

    def get_position(self):
        return self.position

    def get_velocity(self):
        return self.velocity

    def get_orientation(self):
        return self.orientation

    def calibrate_bias(self):
        static_readings = np.array([self.imu.get_acceleration() for _ in range(100)])  # Take 100 readings
        self.bias = np.mean(static_readings, axis=0)  # Calculate the bias


class IMUSensor:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def get_acceleration(self):
        # Simulated IMU data: acceleration in m/s² (assumes DroneKit/MAVLink access)
        ax = self.vehicle.raw_imu.xacc / 1000.0  # Convert milli-g to g
        ay = self.vehicle.raw_imu.yacc / 1000.0
        az = self.vehicle.raw_imu.zacc / 1000.0
        return np.array([ax, ay, az])

    def get_angular_velocity(self):
        # Simulated IMU data: gyroscope in rad/s
        gx = self.vehicle.raw_imu.xgyro
        gy = self.vehicle.raw_imu.ygyro
        gz = self.vehicle.raw_imu.zgyro
        return np.array([gx, gy, gz])


class DroneController:
    def __init__(self, MotionTracker,vehicle):
        self.vehicle = vehicle
        self.MotionTracker = MotionTracker
        self.pid_x = PID(1.0, 0.1, 0.05, setpoint=0)
        self.pid_y = PID(1.0, 0.1, 0.05, setpoint=0)
        self.pid_z = PID(1.0, 0.1, 0.05, setpoint=0)
        self.pid_yaw = PID(1.0, 0.1, 0.05, setpoint=0)

        self.pid_x.output_limits = (-1.0, 1.0)
        self.pid_y.output_limits = (-1.0, 1.0)
        self.pid_z.output_limits = (-1.0, 1.0)
        self.pid_yaw.output_limits = (-0.1, 0.1)  # Yaw rate is slower

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

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            yaw_rate, 0
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def move_forward(self, distance):
        target_dist = self.MotionTracker.position[0]+distance
        while abs(target_dist-self.MotionTracker.position[0])>0.05:
            self.MotionTracker.update()
            velocity_x = self.pid_x(target_dist - self.MotionTracker.position[0])
            self.send_velocity(velocity_x, 0, 0)
            time.sleep(0.1)

    def move_backward(self, distance ):
        target_dist = self.MotionTracker.position[0] - distance
        while (-target_dist + self.MotionTracker.position[0]) > 0.05:
            self.MotionTracker.update()
            velocity_x = self.pid_x(-target_dist + self.MotionTracker.position[0])
            self.send_velocity(-velocity_x, 0, 0)
            time.sleep(0.1)


    def move_left(self, distance):

        target_dist = self.MotionTracker.position[1] - distance
        while (-target_dist + self.MotionTracker.position[1]) > 0.05:
            self.MotionTracker.update()
            velocity_y = self.pid_y(-target_dist + self.MotionTracker.position[1])
            self.send_velocity(0,-velocity_y, 0)
            time.sleep(0.1)


    def move_right(self, distance):
        target_dist = self.MotionTracker.position[1] + distance
        while (target_dist - self.MotionTracker.position[1]) > 0.05:
            self.MotionTracker.update()
            velocity_y = self.pid_y(target_dist - self.MotionTracker.position[1])
            self.send_velocity(0, velocity_y, 0)
            time.sleep(0.1)


    def move_up(self, altitude):

        target_dist = self.MotionTracker.position[2] - altitude
        while (-target_dist + self.MotionTracker.position[2]) > 0.05:
            self.MotionTracker.update()
            velocity_z = self.pid_z(-target_dist + self.MotionTracker.position[2])
            self.send_velocity(0, 0,-velocity_z)
            time.sleep(0.1)


    def move_down(self, altitude):

        target_dist = self.MotionTracker.position[2] + altitude
        while (target_dist - self.MotionTracker.position[2]) > 0.05:
            self.MotionTracker.update()
            velocity_z = self.pid_z(target_dist - self.MotionTracker.position[2])
            self.send_velocity(0, 0, velocity_z)
            time.sleep(0.1)

    def rotate_yaw(self, yaw):
        target_yaw = self.MotionTracker.orientation[2] + yaw
        self.pid_yaw.setpoint = target_yaw
        while abs(self.MotionTracker.orientation[2] - target_yaw) > 0.05:
            self.MotionTracker.update()
            r = self.pid_yaw(self.MotionTracker.orientation[2])
            self.send_velocity(0, 0, 0, r)
            time.sleep(0.1)

    def stop(self):
        self.send_velocity(0, 0, 0, 0)

vehicle = connect('127.0.0.1:14550', wait_ready=True)
imu_sensor = IMUSensor(vehicle)
motion_tracker = MotionTracker(dt=0.1, imu=imu_sensor)

motion_tracker.calibrate_bias()

controller = DroneController(motion_tracker,vehicle)
controller.arm_and_takeoff(10)
controller.move_forward(2)
controller.move_right(2)
controller.move_backward(2)
controller.move_left(2)

time.sleep(0.1)

controller.stop()
