import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from simple_pid import PID  # PID library (install via pip: pip install simple-pid)

# Connect to the Vehicle (assuming SITL)
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# PID controllers for each axis (tuning parameters may need adjustment)
pid_x = PID(1.0, 0.1, 0.05, setpoint=0)  # Forward/Backward PID
pid_y = PID(1.0, 0.1, 0.05, setpoint=0)  # Left/Right PID
pid_z = PID(1.0, 0.1, 0.05, setpoint=0)  # Up/Down PID


# Arm and Takeoff
def arm_and_takeoff(target_altitude):
    print("Arming motors...")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to become armable...")
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


# PID-Controlled Movement
def move_with_pid(target_x, target_y, target_z, duration):
    start_time = time.time()

    # Loop for a specific duration or until convergence
    while time.time() - start_time < duration:
        current_location = vehicle.location.global_relative_frame
        current_x = current_location.lat
        current_y = current_location.lon
        current_z = current_location.alt

        # Calculate error
        error_x = target_x - current_x
        error_y = target_y - current_y
        error_z = target_z - current_z

        # PID control to adjust velocity
        velocity_x = pid_x(error_x)
        velocity_y = pid_y(error_y)
        velocity_z = pid_z(error_z)

        # Send velocity command
        send_ned_velocity(velocity_x, velocity_y, velocity_z)

        print(f"Moving to target: X={target_x}, Y={target_y}, Z={target_z}")
        time.sleep(0.1)


# Send velocity command to the drone (in NED coordinates)
def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # Bitmask for only velocity control
        0, 0, 0,  # Position
        velocity_x, velocity_y, velocity_z,  # Velocity
        0, 0, 0,  # Acceleration (not used)
        0, 0)  # Yaw, Yaw Rate (not used)
    vehicle.send_mavlink(msg)
    vehicle.flush()


# Example PID-controlled diagonal movement
def diagonal_movement():
    target_lat = vehicle.location.global_relative_frame.lat + 0.00001  # Small change in latitude
    target_lon = vehicle.location.global_relative_frame.lon + 0.00001  # Small change in longitude
    target_alt = vehicle.location.global_relative_frame.alt + 2  # Ascend 2 meters

    move_with_pid(target_lat, target_lon, target_alt, duration=10)


# Rotate the Vehicle
def rotate_yaw(target_heading):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        target_heading,
        0,
        1,  # Direction: 1 = clockwise, -1 = counter-clockwise
        1, 0, 0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


# Landing the Vehicle
def land():
    print("Initiating landing...")
    vehicle.mode = VehicleMode("LAND")


# Main Function
if __name__ == "__main__":
    try:
        # Arm and take off to 10 meters
        arm_and_takeoff(9)
        #
        # # Move diagonally with PID control
        # diagonal_movement

        send_ned_velocity(3, 4, 0)
        time.sleep(1)
        send_ned_velocity(-4, 3, 0)
        time.sleep(2)
        send_ned_velocity(0, -4, 0)
        time.sleep(6)
        # # Rotate to a specific heading (yaw)
        # rotate_yaw(90)
        send_ned_velocity(10, 0, 0)
        time.sleep(20)
        # # Land the vehicle
        land()

    except KeyboardInterrupt:
        print("Interrupted. Landing...")
        land()

    finally:
        # Close the vehicle connection
        vehicle.close()
