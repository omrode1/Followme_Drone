from dronekit import connect, VehicleMode
import cv2
import cv2.aruco as aruco
import numpy as np
from picamera2 import Picamera2
from pymavlink import mavutil
import argparse
import time
velocity_scale = 0.5
max_velocity = 2
max_velocity_threshold = 1.5

# Function to transform local coordinates to camera frame
def transform_to_camera_frame(tVec, rVec, local_coordinates):
    rotation_matrix, _ = cv2.Rodrigues(rVec)
    aruco_camera_frame = np.dot(rotation_matrix, local_coordinates.T).T + tVec
    return aruco_camera_frame

# Load camera calibration parameters (intrinsic and distortion)
calib_data_path = "/home/raspberrypi/calibration.npz"
calib_data = np.load(calib_data_path)
mtx, dist = calib_data["mtx"], calib_data["dist"]

#function to calculate distance to travel in x y direction to reach the aruco marker
def calculate_distance_to_travel (dist_x, dist_y, dist_z, rVec, tVec, distance, theta, theta_deg, res_dist, time, velocity_x, velocity_y):
    distance = np.sqrt(tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2)
    #calculate angle(theta) between dist_x and and distance 
    theta = np.arctan(tVec[i][0][0]/distance)
    theta_deg = np.degrees(theta)

    #calculate distance to travel in x direction
    dist_x = distance * np.cos(theta_deg)

    #calculate distance to travel in y direction
    dist_y = distance * np.tan(theta_deg)

    #convert distance to meters
    dist_x = dist_x * 0.01
    dist_y = dist_y * 0.01

    #calculate the resulant distance to travel in xy plane 
    res_dist = np.sqrt(dist_x**2 + dist_y**2)

    #time is kept constant for the vehicle to travel the distance
    time = 5

    #calculate the velocity in x direction
    velocity_x = dist_x/time

    #calculate the velocity in y direction
    velocity_y = dist_y/time

    #apply scaling factor to the velocity
    velocity_x *= velocity_scale
    velocity_y *= velocity_scale

    #limit the velocity to max_velocity
    velocity_x = min(velocity_x, max_velocity)
    velocity_y = min(velocity_y, max_velocity)

    return velocity_x, velocity_y


#function to check if the drones velocity exceeds the threshold
def check_velocity_exceeds_threshold(velocity_x, velocity_y, threshold):
   return abs(velocity_x) > threshold or abs(velocity_y) > threshold


# Initialize the PiCamera
piCam = Picamera2()
piCam.preview_configuration.main.size = (1280, 720)
piCam.preview_configuration.main.format = "RGB888"
piCam.preview_configuration.align()
piCam.configure("preview")
piCam.start()

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='/dev/ttyAMA0')
args = parser.parse_args()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=57600, wait_ready=True)

velocity_y = 0
velocity_z = 0

MARKER_SIZE = 14.5  # centimeters

marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters_create()

def get_camera_gps():
    latitude = vehicle.location.global_frame.lat
    longitude = vehicle.location.global_frame.lon
    altitude = vehicle.location.global_relative_frame.alt
    print("latitude: ",latitude,"longitude: ", longitude,"altitude: ", altitude)
    return [latitude, longitude, altitude]

def calculate_aruco_gps(tVec, camera_gps):
    aruco_lat = camera_gps[0] + tVec[i][0][0] * 1e-7
    aruco_lon = camera_gps[1] + tVec[i][0][1] * 1e-7
    aruco_alt = camera_gps[2] + tVec[i][0][2]

    return aruco_lat, aruco_lon, aruco_alt

# Function to arm and takeoff
def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Armed for 5 seconds before takeoff")
    time.sleep(5)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

arm_and_takeoff(2)

desired_loop_frequency = 10  # Set your desired loop frequency in Hz

while True:
    loop_start_time = time.time()

    
    frame = piCam.capture_array()
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers
    )

    if marker_corners:
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
            marker_corners, MARKER_SIZE, mtx, dist
        )

        total_markers = range(0, marker_IDs.size)

        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            if 13 in ids:
                cv2.polylines(
                    frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
                )

                corners = corners.reshape(4, 2)
                corners = corners.astype(int)

                # Calculate centroid
                centroid_x = int(np.mean(corners[:, 0]))
                centroid_y = int(np.mean(corners[:, 1]))

                # Draw centroid
                cv2.circle(frame, (centroid_x, centroid_y), 5, (255, 255, 0), -1)

                top_right = corners[0].ravel()
                bottom_right = corners[2].ravel()

                distance = np.sqrt(
                    tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
                )
                #draw grid on the frame (3x3)
                for i in range(0, 3):
                    cv2.line(frame, (0, i * 240), (1280, i * 240), (0, 0, 255), 2)
                    cv2.line(frame, (i * 426, 0), (i * 426, 720), (0, 0, 255), 2)

                #print velocity and distance on camera frame
                cv2.putText(
                    frame,
                    f"Distance: {round(distance, 2)}",
                    (10, 30),
                    cv2.FONT_HERSHEY_PLAIN,
                    1.3,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                    
                camera_gps = get_camera_gps()
                aruco_lat, aruco_lon, aruco_alt = calculate_aruco_gps(tVec, camera_gps)

                # Function to transform local coordinates to camera frame
                aruco_local_coordinates = np.array([0, 0, distance])
                aruco_camera_frame = transform_to_camera_frame(tVec[i], rVec[i], aruco_local_coordinates)

                # Add camera's GPS coordinates to get global coordinates in camera frame
                aruco_global_camera_frame = aruco_camera_frame + np.array([camera_gps])

                # Convert camera frame coordinates to global GPS coordinates (optional)
                aruco_global_gps = aruco_global_camera_frame.squeeze()  # Assuming 1D array

                print("ARUCO Marker Global GPS Coordinates:", aruco_lat, aruco_lon, aruco_alt)

                # Your existing code for sending MAVLink messages goes here

            if distance < 200:             
            
                velocity_x, velocity_y = calculate_distance_to_travel(0, 0, 0, rVec, tVec, distance, 0, 0, 0, 0, 0)
                
                if check_velocity_exceeds_threshold(velocity_x, velocity_y, max_velocity_threshold):
                    print("velocity exceeds threshold, switching to loiter mode")
                    vehicle.mode = VehicleMode("LOITER")

                else:
                 print("Aruco marker is very close ,vehicle is moving backward to ", aruco_lat, aruco_lon, aruco_alt)
                 msg = vehicle.message_factory.set_position_target_local_ned_encode(
                 0,  # time_boot_ms (not used)
                 0, 0,  # target system, target component
                 mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
                 0b0000111111000111,  # type_mask
                 aruco_lat, aruco_lon, aruco_alt,  # x, y, z positions 
                 velocity_x , velocity_y, 0,  # m/s
                 0, 0, 0,  # x, y, z acceleration
                 0, 0)
                 vehicle.send_mavlink(msg)

            if distance > 200 and distance < 210:   #50 60
                
                velocity_x, velocity_y = calculate_distance_to_travel(0, 0, 0, rVec, tVec, distance, 0, 0, 0, 0, 0)

                print("Aruco marker is in correct position, vehicle will remain stationary")
               
                msg = vehicle.message_factory.set_position_target_local_ned_encode(
                0,  # time_boot_ms (not used)
                0, 0,  # target system, target component
                mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
                0b0000111111000111,  # type_mask
                0, 0, 0,  # x, y, z positions (not used)
                0, 0, 0,  # m/s
                0, 0, 0,  # x, y, z acceleration
                0, 0)
                vehicle.send_mavlink(msg)   


            if distance > 210:   

                velocity_x, velocity_y = calculate_distance_to_travel(0, 0, 0, rVec, tVec, distance, 0, 0, 0, 0, 0)

                print("Aruco marker is very far ,vehicle is moving forward to ", aruco_lat, aruco_lon, aruco_alt)

                if check_velocity_exceeds_threshold(velocity_x, velocity_y, max_velocity_threshold):
                    print("velocity exceeds threshold, switching to loiter mode")
                    vehicle.mode = VehicleMode("LOITER")

                else:
                    msg = vehicle.message_factory.set_position_target_local_ned_encode(
                    0,  # time_boot_ms (not used)
                    0, 0,  # target system, target component
                    mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
                    0b0000111111000111,  # type_mask
                    aruco_lat, aruco_lon, aruco_alt,  # x, y, z positions (not used)
                    velocity_x, velocity_y, 0,  # m/s velocity x y z
                    0, 0, 0,  # x, y, z acceleration
                    0, 0)
                    vehicle.send_mavlink(msg)

            else:
                #send command to vehicle on 1 hz cycle
                for x in range(0, time, 1):
                    vehicle.send_mavlink(msg)
                    time.sleep(1)
            
                point = cv2.drawFrameAxes(frame, mtx, dist, rVec[i], tVec[i], 4, 4)
                cv2.putText(
                    frame,
                    f"id: {ids[0]} Dist: {round(distance, 2)}",
                    tuple(top_right),
                    cv2.FONT_HERSHEY_PLAIN,
                    1.3,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    frame,
                    f"x:{round(tVec[i][0][0], 1)} y: {round(tVec[i][0][1], 1)} ",
                    tuple(bottom_right),
                    cv2.FONT_HERSHEY_PLAIN,
                    1.0,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

    cv2.imshow("ArUco Marker Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    loop_execution_time = time.time() - loop_start_time

    loop_frequency = 1 / loop_execution_time
    print(f"Loop Frequency: {loop_frequency} Hz")

    # Introduce a delay to control the loop rate
    loop_delay = 1 / desired_loop_frequency
    time.sleep(max(0, loop_delay - loop_execution_time))

# Release resources
cv2.destroyAllWindows()
piCam.stop()
vehicle.close()

