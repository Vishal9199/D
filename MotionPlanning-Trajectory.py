from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

# Connect to the vehicle and start the mission planner.
connection_string = 'udp:127.0.0.1:14550'
vehicle = connect(connection_string)
master = mavutil.mavlink_connection(connection_string)
master.wait_heartbeat()

# Set the vehicle mode to GUIDED and arm the vehicle.
vehicle.mode = VehicleMode('GUIDED')
vehicle.armed = True
while not vehicle.armed:
    time.sleep(1)

# Set the target altitude and take off.
target_altitude = 10 # meters
vehicle.simple_takeoff(target_altitude)

# Wait for the vehicle to reach the target altitude.
while True:
    altitude = vehicle.location.global_relative_frame.alt
    if altitude >= target_altitude * 0.95:
        break
    time.sleep(1)

# Plan a trajectory consisting of three waypoints.
# You can use the Mission Planner to plan the trajectory and export it as a .txt file.
waypoints_file = 'path/to/waypoints.txt'
with open(waypoints_file) as f:
    waypoints = f.readlines()
waypoints = [waypoint.strip().split(',') for waypoint in waypoints]
waypoints = [[float(waypoint[0]), float(waypoint[1]), float(waypoint[2]), float(waypoint[3])] for waypoint in waypoints]

# Upload the waypoints to the vehicle.
mission_items = []
for i, waypoint in enumerate(waypoints):
    mission_item = mavutil.mavlink.MAVLink_mission_item_message(
        master.target_system,
        master.target_component,
        i+1,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 0, 0, 0, 0, 0,
        waypoint[0], waypoint[1], waypoint[2]
    )
    mission_items.append(mission_item)

mavutil.mavlink_utils.write_list(master, mission_items)

# Arm the vehicle and execute the mission.
vehicle.mode = VehicleMode('AUTO')
while True:
    if vehicle.mode.name == 'AUTO':
        break
    time.sleep(1)

# Wait for the vehicle to complete the mission.
while True:
    if vehicle.mode.name == 'GUIDED':
        break
    time.sleep(1)

# Land the vehicle.
vehicle.mode = VehicleMode('LAND')

# Wait for the vehicle to land.
while True:
    if not vehicle.armed:
        break
    time.sleep(1)

# Close the vehicle connection.
vehicle.close()
