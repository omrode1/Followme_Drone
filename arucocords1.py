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

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='/dev/ttyAMA0')
args = parser.parse_args()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=57600, wait_ready=True)

velocity_y = 0
velocity_z = 0

MARKER_SIZE = 10  # centimeters

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

while True:
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

            fb = 0
            if distance < 200:
                print("Aruco marker is very close")
#                fb = (distance - 200)/100 #50
                fb = -1.0
                print(fb)
                msg = vehicle.message_factory.set_position_target_local_ned_encode(
                0,  # time_boot_ms (not used)
                0, 0,  # target system, target component
                mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
                0b0000111111000111,  # type_mask
                aruco_lat, aruco_lon, aruco_alt,  # x, y, z positions (not used)
                fb, velocity_y, velocity_z,  # m/s
                0, 0, 0,  # x, y, z acceleration
                0, 0)
                vehicle.send_mavlink(msg)
#speed = 5 * (distance - 100)
                #print(speed)
# existing distance, remaining distance, speed, error
            if distance > 200 and distance < 210:   #50 60
                print("Aruco marker is in correct position")
                fb= 0
                print(fb)
                msg = vehicle.message_factory.set_position_target_local_ned_encode(
                0,  # time_boot_ms (not used)
                0, 0,  # target system, target component
                mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
                0b0000111111000111,  # type_mask
                aruco_lat, aruco_lon, aruco_alt,  # x, y, z positions (not used)
                fb, velocity_y, velocity_z,  # m/s
                0, 0, 0,  # x, y, z acceleration
                0, 0)
                vehicle.send_mavlink(msg)
            if distance > 210:    # 60
                print("Aruco marker is very far")
#                fb = (distance - 210)/100
                fb = 1.0
                print(fb)
                msg = vehicle.message_factory.set_position_target_local_ned_encode(
                0,  # time_boot_ms (not used)
                0, 0,  # target system, target component
                mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
                0b0000111111000111,  # type_mask
                aruco_lat, aruco_lon, aruco_alt,  # x, y, z positions (not used)
                fb, velocity_y, velocity_z,  # m/s
                0, 0, 0,  # x, y, z acceleration
                0, 0)
                vehicle.send_mavlink(msg)
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

# Release resources
cv2.destroyAllWindows()
piCam.stop()
vehicle.close()
