from dronekit import connect, VehicleMode
import cv2
import cv2.aruco as aruco
import numpy as np
from picamera2 import Picamera2
from pymavlink import mavutil
import argparse
import time

# Function to transform local coordinates to camera frame
def transform_to_camera_frame(tVec, rVec, local_coordinates):
    rotation_matrix, _ = cv2.Rodrigues(rVec)
    aruco_camera_frame = np.dot(rotation_matrix, local_coordinates.T).T + tVec
    return aruco_camera_frame

# Load camera calibration parameters (intrinsic and distortion)
calib_data_path = "/home/raspberrypi/calibration.npz"
calib_data = np.load(calib_data_path)
mtx, dist = calib_data["mtx"], calib_data["dist"]

# Initialize the PiCamera
piCam = Picamera2()
piCam.preview_configuration.main.size = (1280, 720)
piCam.preview_configuration.main.format = "RGB888"
piCam.preview_configuration.align()
piCam.configure("preview")
piCam.start()

# Connect to the Vehicle
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='/dev/ttyAMA0')
args = parser.parse_args()
print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=57600, wait_ready=True)

velocity_y = 0
velocity_z = 0

MARKER_SIZE = 10  # centimeters

# Load the predefined dictionary
marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters_create()

# Function to get camera GPS location
def get_camera_gps():
    latitude = vehicle.location.global_frame.lat
    longitude = vehicle.location.global_frame.lon
    altitude = vehicle.location.global_relative_frame.alt
    print("latitude: ", latitude, "longitude: ", longitude, "altitude: ", altitude)
    return [latitude, longitude, altitude]

# Function to calculate ArUco GPS coordinates
def calculate_aruco_gps(tVec, camera_gps, i):
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

#function to calculate gimbal angles (pitch, roll, yaw)
def calculate_gimbal_angles(tVec, rVec, camera_gps, i):
    pitch = 0
    roll = 0
    yaw = 0 
    return pitch, roll, yaw

desired_loop_frequency = 10  # Set your desired loop frequency in Hz

# To display the cv2 overlays on the video feed
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

                camera_gps = get_camera_gps()
                aruco_lat, aruco_lon, aruco_alt = calculate_aruco_gps(tVec, camera_gps, i)

                # Function to transform local coordinates to camera frame
                aruco_local_coordinates = np.array([0, 0, distance])
                aruco_camera_frame = transform_to_camera_frame(tVec[i], rVec[i], aruco_local_coordinates)

                # Add camera's GPS coordinates to get global coordinates in camera frame
                aruco_global_camera_frame = aruco_camera_frame + np.array([camera_gps])

                # Convert camera frame coordinates to global GPS coordinates (optional)
                aruco_global_gps = aruco_global_camera_frame.squeeze()  # Assuming 1D array

                print("ARUCO Marker Global GPS Coordinates:", aruco_lat, aruco_lon, aruco_alt)

                # Calculate Gimbal angles
                pitch, roll, yaw = calculate_gimbal_angles(tVec[i], rVec[i])

                # Send MAVLink gimbal commands to keep the camera facing the marker
                gimbal_msg = vehicle.message_factory.command_long_encode(
                    0, 0,  # target system, target component
                    mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,  # command
                    0,  # confirmation
                    # Gimbal control: pitch, roll, yaw, altitude, latitude, longitude
                    pitch, roll, yaw, 0, aruco_lat, aruco_lon,
                    0, 0, 0,  # Speed, duration, and command
                )
                vehicle.send_mavlink(gimbal_msg)

                # Print Gimbal angles for debugging
                print("Gimbal Angles - Pitch: {}, Roll: {}, Yaw: {}".format(pitch, roll, yaw))
                 
                # Your existing code for sending MAVLink messages goes here
                vehicle.send_mavlink(gimbal_msg)


                # For movement in x direction (forward/backward)
                fb = 0

                if distance < 200:
                    print(f"Aruco marker is very close, vehicle is moving to {aruco_lat}, {aruco_lon}, {aruco_alt}")

                    # Move forward without adjusting yaw
                    vehicle.simple_goto(
                        location=vehicle.location.global_relative_frame,
                        latitude=aruco_lat,
                        longitude=aruco_lon,
                        altitude=aruco_alt,
                    )
                    

                elif 200 <= distance < 210:
                    print("Aruco marker is in correct position, vehicle will remain stationary")
                    fb = 0
                    velocity_y = 0  # Ensure no lateral movement

                elif distance >= 210:
                    print(f"Aruco marker is very far, vehicle is moving forward to {aruco_lat}, {aruco_lon}, {aruco_alt}")

                    # Move forward without adjusting yaw
                    vehicle.simple_goto(
                        location=vehicle.location.global_relative_frame,
                        latitude=aruco_lat,
                        longitude=aruco_lon,
                        altitude=aruco_alt,
                    )

                else:
                    fb = 0

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