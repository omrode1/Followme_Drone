from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from picamera2 import Picamera2
import cv2
from cv2 import aruco
import argparse
import time

# Initialize the PiCamera
picam = Picamera2()
picam.preview_configuration.main.size = (1280, 720)
picam.preview_configuration.main.format = "RGB888"
picam.preview_configuration.align()
picam.configure("preview")
picam.start()

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='/dev/ttyAMA0')
args = parser.parse_args()

# Connection string for the drone
print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=57600, wait_ready=True)

# Function to arm and takeoff
def arm_and_takeoff(target_altitude):
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Armed for 5 seconds before takeoff")
    time.sleep(5)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:  # Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

# Function for visual servoing
def visual_servo():
    while True:
        # Use the drone's camera to capture frames
        frame = picam.capture_array()

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Create ArUco dictionary and parameters
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
        parameters = aruco.DetectorParameters_create()

        # Detect ArUco markers
        corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

        if ids is not None:
            # Assuming the first marker is the target
            target_id = ids[0]
            target_corners = corners[0]

            # Compute the image Jacobian based on the target ArUco marker
            image_jacobian = compute_image_jacobian(target_corners)

            # Visual servoing control signal based on the image Jacobian
            control_signal = compute_control_signal(image_jacobian)

            # Move the drone based on the control signal
            move_drone(control_signal)

        # Display the frame
        cv2.imshow('Frame', frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Close OpenCV window
    cv2.destroyAllWindows()

# Function to compute the image Jacobian
def compute_image_jacobian(target_corners):
    # Implement the computation of the image Jacobian based on the target ArUco marker corners
    # You may need to use the actual camera parameters and the drone's pose information for accurate computation
    # For simplicity, this is a placeholder function

    # Placeholder: Return a dummy identity matrix as the image Jacobian
    return np.eye(4)

# Function to compute the control signal based on the image Jacobian
def compute_control_signal(image_jacobian):
    # Placeholder: Compute the control signal based on the image Jacobian
    # This can involve selecting specific rows or columns of the Jacobian and scaling factors
    # Adjust this part based on your camera calibration and desired control strategy
    return np.zeros(6)

# Function to move the drone based on the control signal
def move_drone(control_signal):
    # Placeholder: Move the drone based on the control signal
    # Implement the actual control logic based on your control strategy
    pass

# Arm and takeoff to 5 meters
arm_and_takeoff(2)

# Visual servoing to follow ArUco marker
visual_servo()

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

print("Completed")

