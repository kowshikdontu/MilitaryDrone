import asyncio
import websockets
import json
from basic_func import *
from dronekit import connect, VehicleMode


vehicle = connect('127.0.0.1:14550', wait_ready=True)
motion_tracker = MotionTracker(vehicle, 0.1)
controller = DroneController(motion_tracker, vehicle)
vehicle.mode = VehicleMode("GUIDED")
def instructor(command,prev):
    if command[0] == "stop":
        controller.stop()
    elif command[0] == "left":
        vel = 0.1 if len(command) == 1 else int(command[1])
        controller.send_velocity(vel, 0, 0)
    elif command[0] == "Right":
        vel = 0.1 if len(command) == 1 else int(command[1])
        controller.send_velocity(-vel, 0, 0)
    elif command[0] == "up":
        vel = 0.1 if len(command) == 1 else int(command[1])
        controller.send_velocity(0, 0, vel)
    elif command[0] == "down":
        vel = 0.1 if len(command) == 1 else int(command[1])
        controller.send_velocity(0, 0, -vel)
    elif command[0] == "rotate":
        controller.condition_yaw(360)
    elif command[0] == "revolve":
        radius = 2 if len(command) == 1 else int(command[1])
        controller.revolve_around_point_ned(radius)
    elif "clock" in command:
        controller.move_in_clock_direction(int(command[0]))
    elif command[0] == "speed":
        speed = int(command[-1]) if len(command)>1 else int(prev[-1])+1
        new_command =prev[:-1]+str(speed)
        instructor(new_command,new_command)


async def handle_commands(websocket, path):
    url = "ws://<ground-station-ip>:8765"
    async with websockets.connect(url) as websocket:
        prev = None
        while True:
            data = await websocket.recv()
            command = json.loads(data)
            command = command.split()
            if command[0]=="over":
                instructor("stop",prev)
                break
            instructor(command,prev)
            prev = command
