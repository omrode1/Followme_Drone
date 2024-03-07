import dronekit
import time
import math
import argparse
from pymavlink import mavutil
import numpy as np
import cv2
import cv2.aruco as aruco
import sys
from picamera2 import PiCamera


#listner for the drone
def listener():
    vehicle = dronekit.connect('udp:
    vehicle.wait_ready(True)
    print("Connected to vehicle on: %s" % vehicle.connection_string())
    print("Mode: %s" % vehicle.mode.name)
    print("Armed: %s" % vehicle.armed)
    print("Location: %s" % vehicle.location.global_frame)
    print("Attitude: %s" % vehicle.attitude)
    print("Velocity: %s" % vehicle.velocity)
    print("GPS: %s" % vehicle.gps_0)
    print("Groundspeed: %s" % vehicle.groundspeed)
    print("Airspeed: %s" % vehicle.airspeed)
    print("Battery: %s" % vehicle.battery)
    print("EKF OK?: %s" % vehicle.ekf_ok)
    print("Last Heartbeat: %s" % vehicle.last_heartbeat)
    print("Rangefinder: %s" % vehicle.rangefinder)
    print("Rangefinder distance: %s" % vehicle.rangefinder.distance)
    print("Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
    print("Heading: %s" % vehicle.heading)
    print("Is Armable?: %s" % vehicle.is_armable)

    while True:
        print("Mode: %s" % vehicle.mode.name)
        print("Armed: %s" % vehicle.armed)
        print("Location: %s" % vehicle.location.global_frame)
        print("Attitude: %s" % vehicle.attitude)
        print("Velocity: %s" % vehicle.velocity)
        print("GPS: %s" % vehicle.gps_0)
        print("Groundspeed: %s" % vehicle.groundspeed)
        print("Airspeed: %s" % vehicle.airspeed)
        print("Battery: %s" % vehicle.battery)
        print("EKF OK?: %s" % vehicle.ekf_ok)
        print("Last Heartbeat: %s" % vehicle.last_heartbeat)
        print("Rangefinder: %s" % vehicle.rangefinder)
        print("Rangefinder distance: %s" % vehicle.rangefinder.distance)
        print("Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
        print("Heading: %s" % vehicle.heading)
        print("Is Armable?: %s" % vehicle.is_armable)
        time.sleep(1)
    vehicle.close()

#function to arm the drone
def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors")
    vehicle.mode = dronekit.VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

#function to send velocity commands to the drone
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0, 0, 0,
        velocity_x, velocity_y, velocity_z,
        0, 0, 0,
        0, 0)
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

#function to move the drone forward
def move_forward():
    print("Moving forward")
    send_ned_velocity(2, 0, 0, 5)

#function to move the drone backward
def move_backward():
    print("Moving backward")
    send_ned_velocity(-2, 0, 0, 5)

#function to move the drone left
def move_left():
    print("Moving left")
    send_ned_velocity(0, -2, 0, 5)

#function to move the drone right
def move_right():
    print("Moving right")
    send_ned_velocity(0, 2, 0, 5)

#function to move the drone up
def move_up():
    print("Moving up")
    send_ned_velocity(0, 0, -2, 5)

#function to move the drone down
def move_down():
    print("Moving down")
    send_ned_velocity(0, 0, 2, 5)

#function to rotate the drone clockwise
def rotate_clockwise():
    print("Rotating clockwise")
    send_ned_velocity(0, 0, 0, 5)

#function to rotate the drone counter-clockwise
def rotate_counter_clockwise():
    print("Rotating counter-clockwise")
    send_ned_velocity(0, 0, 0, 5)

#function to land the drone
def land():
    print("Landing")
    vehicle.mode = dronekit.VehicleMode("LAND")

#function to fly the drone in a square path
def square():
    move_forward()
    move_right()
    move_backward()
    move_left()

#function to fly the drone in a circle path
def circle():
    move_forward()
    rotate_clockwise()
    move_forward()
    rotate_clockwise()
    move_forward()
    rotate_clockwise()
    move_forward()
    rotate_clockwise()

#initilaize the picamera 
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
camera.preview_configuration.main.size(1280,720)
camera.preview_configuration.main.format('h264')
camera.preview_configuration.main.start()

parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect', default='/dev/ttyAMA0')
args = parser.parse_args()
connection_string = args.connect

#follow the aruco marker
def follow_aruco():
    while True:
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            image = frame.array
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
            parameters = aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            if ids is not None:
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
                (rvec - tvec).any()
                aruco.drawAxis(image, mtx, dist, rvec, tvec, 0.1)
                aruco.drawDetectedMarkers(image, corners)
                print("tvec: ", tvec)
                print("rvec: ", rvec)
                if tvec[0][0][0] > 0.1:
                    move_right()
                elif tvec[0][0][0] < -0.1:
                    move_left()
                elif tvec[0][0][2] > 0.1:
                    move_forward()
                elif tvec[0][0][2] < -0.1:
                    move_backward()
            cv2.imshow('image', image)
            key = cv2.waitKey(1) & 0xFF
            rawCapture.truncate(0)
            if key == ord("q"):
                break
                
#function to land the drone
def land():
    print("Landing")
    vehicle.mode = dronekit.VehicleMode("LAND")


    
    
