from dronekit import connect, VehicleMode
import cv2
import cv2.aruco as aruco
import numpy as np
from picamera2 import Picamera2
from pymavlink import mavutil
import argparse
import time
from dronekit import LocationGlobalRelative

#all the functions and variables are defined in this file
velocity_x = 0
velocity_y = 0
duration = 0

# Function to transform local coordinates to camera frame
def transform_to_camera_frame(tVec, rVec, local_coordinates):
    rotation_matrix, _ = cv2.Rodrigues(rVec)
    aruco_camera_frame = np.dot(rotation_matrix, local_coordinates.T).T + tVec
    return aruco_camera_frame

# Load camera calibration parameters (intrinsic and distortion)
calib_data_path = "/home/pi/calibration.npz"
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

MARKER_SIZE = 14.5  # centimeters

marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters_create()

#function to calculate distance between the aruco marker and the camera
def calculate_distance(tVec):
    distance = np.sqrt(tVec[0][0]**2 + tVec[0][1]**2 + tVec[0][2]**2)
    print("Distance: ", distance)
    return distance

#function to calculate the angle between the aruco marker and the camera
def calculate_angle(tVec):
    theta = np.arctan(tVec[0][0]/tVec[0][2]) #angle in radians
    print("Theta: ", np.degrees(theta))
    return np.degrees(theta) #angle in degrees

#function to calculate the intended travel distance of the vehicle in y direction
def calculate_travel_distance_y(distance, angle):
    travel_distance_y = distance * np.sin(angle)
    print("Travel Distance Y: ", travel_distance_y)
    return travel_distance_y

#function to calculate the intended travel distance of the vehicle in x direction
def calculate_travel_distance_x(distance, angle):
    travel_distance_x = distance * np.cos(angle)
    print("Travel Distance X: ", travel_distance_x)
    return travel_distance_x

#function to calculate the time required to travel the intended distance
def calculate_time(travel_distance_x, travel_distance_y, velocity):
    velocity = 2 #velocity should be 2 m/s at max to avoid overshooting the target
    time_x = travel_distance_x / velocity
    time_y = travel_distance_y / velocity
    print("Time: ", time_x, time_y)
    return time_x

#function to calculate the the travel velocity in y direction (velocity should be 2 m/s at max to avoid overshooting the target)
def calculate_velocity_y(travel_distance_y, time_y):
    velocity_y = travel_distance_y / time_y
    print("Velocity Y: ", velocity_y)
    velocity_y = np.clip(velocity_y, -2, 2)
    return velocity_y

#function to calculate the the travel velocity in x direction (velocity should be 2 m/s at max to avoid overshooting the target)
def calculate_velocity_x(travel_distance_x, time_x):
    velocity_x = travel_distance_x / time_x
    print("Velocity X: ", velocity_x)
    velocity_x = np.clip(velocity_x, -2, 2)
    return velocity_x


#function to get camera gps location
def get_camera_gps():
    latitude = vehicle.location.global_frame.lat
    longitude = vehicle.location.global_frame.lon
    altitude = vehicle.location.global_relative_frame.alt
    print("latitude: ", latitude, "longitude: ", longitude, "altitude: ", altitude)
    return [latitude, longitude, altitude]

def calculate_aruco_gps(tVec, camera_gps, i):
    aruco_lat = camera_gps[0] + tVec[i][0][0] * 1e-7
    aruco_lon = camera_gps[1] + tVec[i][0][1] * 1e-7
    aruco_alt = camera_gps[2] + tVec[i][0][2]
    return aruco_lat, aruco_lon, aruco_alt

# function to calculate duration of travel
def calculate_duration(time_x, time_y, desired_total_duration):
    
    #calc the scaling factor to adjust the duration
    duration_scaling_factor = desired_total_duration / max(time_x, time_y)

    #apply tthe scaling factor to individual time components
    duration_x = time_x * duration_scaling_factor
    duration_y = time_y * duration_scaling_factor

    #use the maximum of the scaled time components as the duration
    duration = max(duration_x, duration_y)

    return int(duration)


# Function to send global velocity
def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111, 0, 0, 0,
        velocity_x, velocity_y, velocity_z,
        0, 0, 0, 0, 0)
    for _ in range(duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

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

arm_and_takeoff(5)

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

                camera_gps = get_camera_gps()
                aruco_lat, aruco_lon, aruco_alt = calculate_aruco_gps(tVec, camera_gps, i)

                # calculate desired yaw
                desired_yaw = np.arctan2(aruco_lat - vehicle.location.global_frame.lat, aruco_lon - vehicle.location.global_frame.lon)
                desired_yaw_degrees = np.degrees(desired_yaw)
                print("Desired Yaw:", desired_yaw_degrees)  # yaw in terms of degrees
                yaw_rate = 15  # yaw rate in terms of degree/s
                # Your existing code for sending MAVLink messages goes here

                # Adjust the gain based on your experiment
                velocity_z = 0

                # Inside the loop
                if distance < 200:
                    print(f"Aruco marker is very close, adjusting camera orientation.")
                    
                    
                elif 200 <= distance < 210:
                    print("Aruco marker is in the correct position, the vehicle will remain stationary.")

                elif distance >= 210:
                    
                    #calculate angle
                    angle = calculate_angle(tVec[i])

                    #calculate the travel distances in x and y directions
                    travel_distance_x = calculate_travel_distance_x(distance, angle)
                    travel_distance_y = calculate_travel_distance_y(distance, angle)

                    #calculate the time required to travel the intended distance
                    time_x = calculate_time(travel_distance_x, travel_distance_y, velocity_x)

                    #update the velocity and duration based on the calculation
                    velocity_x = calculate_velocity_x(travel_distance_x, time_x)
                    velocity_y = calculate_velocity_y(travel_distance_y, time_x)
                    duration = calculate_duration(time_x, time_x, 5)
                    
                    #print(f"Aruco marker is very far, the vehicle is moving forward to {aruco_lat}, {aruco_lon}, {aruco_alt}")
                    print("velocity_x",velocity_x,"velocity_y", velocity_y,"velocity_z", velocity_z,"duration", duration)
                    
                    # Send velocity commands for smooth drone movements
                    send_global_velocity(velocity_x, velocity_y, velocity_z, duration)

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
    #print(f"Loop Frequency: {loop_frequency} Hz")

    # Introduce a delay to control the loop rate
    loop_delay = 1 / desired_loop_frequency
    time.sleep(max(0, loop_delay - loop_execution_time))

# Release resources
cv2.destroyAllWindows()
piCam.stop()
vehicle.close()
