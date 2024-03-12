from dronekit import connect, VehicleMode
import cv2
import cv2.aruco as aruco
import numpy as np
from picamera2 import Picamera2
from pymavlink import mavutil
import argparse
import timefrom dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command, Vehicle
from logging import debug
from logging import debug
from simple_pid import PID
import time
import pyzed.sl as sl
import numpy as np
import cv2
import argparse
import sys
import math
import os
import serial, time 
from pymavlink import mavutil
import tflite_runtime.interpreter as tflite
from modules import drone


ser = None 
vehicle = None
net = None
camera = None
USE_PID_YAW = True
USE_PID_ROLL = False
cams = []

MAX_SPEED = 3          # m/s
MAX_YAW = 15           # degrees/s

#PID values
P_YAW = 0.1
I_YAW = 0
D_YAW = 0

P_ROLL = 0.1
I_ROLL = 0
D_ROLL = 0

#initialize the PID controllers
control_loop_active = True
pidYaw = None
pidRoll = None
movementYawAngle = 0
movementRollAngle = 0
inputValueYaw = 0
inputValueVelocityX = 0
control_loop_active = True
flight_altitude = 4

debug_yaw = None
debug_velocity = None

def create_camera(camera_index):
    cams.append(cv2.VideoCapture(camera_index))

def get_image_size(camera_id):
    width = cams [camera_id].get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cams [camera_id].get(cv2.CAP_PROP_FRAME_HEIGHT)
    return width, height

def get_video(camera_id):
    ret, frame = cams[camera_id].read()
    if ret:
        return frame
    else:
        return None

def close_camera():
    for cam in cams:
        cam.release()

if __name__ == "__main__":
    create_camera(0)
    
    while True:
        img = get_video(0)
        if img is not None:
            cv2.imshow("Camera 1", img)
            cv2.waitKey(1)

    close_camera()

    

def configure_PID(control):
    global pidRoll, pidYaw

    """creates a new pid object depending on whether or not the oid or p is used"""
    print("Configuring control")


    if control == 'PID':
        pidYaw = PID(P_YAW, I_YAW, D_YAW, setpoint=0)       # I = 0.001
        pidYaw.output_limits = (-MAX_YAW, MAX_YAW)          # PID Range
        pidRoll = PID(P_ROLL, I_ROLL, D_ROLL, setpoint=0)   # I = 0.001
        pidRoll.output_limits = (-MAX_SPEED, MAX_SPEED)     # PID Range
        print("Configuring PID")
    else:
        pidYaw = PID(P_YAW, 0, 0, setpoint=0)               # I = 0.001
        pidYaw.output_limits = (-MAX_YAW, MAX_YAW)          # PID Range
        pidRoll = PID(P_ROLL, 0, 0, setpoint=0)             # I = 0.001
        pidRoll.output_limits = (-MAX_SPEED, MAX_SPEED)     # PID Range
        print("Configuring P")


#connect drone
def connect_drone(drone_location):
    drone.connect_drone(drone_location) #'/dev/ttyACM0'

def getMovementYawAngle():
    return movementYawAngle

def setXdelta(XDelta):
    global inputValueYaw
    inputValueYaw = XDelta

def getMovementVelocityXCommand():
    return movementRollAngle

def setZDelta(ZDelta):
    global inputValueVelocityX
    inputValueVelocityX = ZDelta

def set_system_state(current_state):
    global state
    state = current_state

def set_flight_altitude(alt):
    global flight_altitude
    flight_altitude = alt
# end control functions

#drone functions
def arm_and_takeoff(max_height):
    drone.arm_and_takeoff(max_height)

def land():
    drone.land()

def print_drone_report():
    print(drone.get_EKF_status())
    print(drone.get_battery_info())
    print(drone.get_version())
#end drone functions

def initialize_debug_logs(DEBUG_FILEPATH):
    global debug_yaw, debug_velocity
    debug_yaw = open(DEBUG_FILEPATH + "_yaw.txt", "a")
    debug_yaw.write("P: I: D: Error: command:\n")

    debug_velocity = open(DEBUG_FILEPATH + "_velocity.txt", "a")
    debug_velocity.write("P: I: D: Error: command:\n")

def debug_writer_YAW(value):
    global debug_yaw
    debug_yaw.write(str(0) + "," + str(0) + "," + str(0) + "," + str(inputValueYaw) + "," + str(value) + "\n")

def debug_writer_ROLL(value):
    global debug_velocity
    debug_velocity.write(str(0) + "," + str(0) + "," + str(0) + "," + str(inputValueYaw) + "," + str(value) + "\n")

def control_drone():
    global movementYawAngle, movementRollAngle
    if inputValueYaw == 0:
        drone.send_movement_command_YAW(0)
    else:
        movementYawAngle = (pidYaw(inputValueYaw) * -1)
        drone.send_movement_command_YAW(movementYawAngle)
        debug_writer_YAW(movementYawAngle)

    if inputValueVelocityX == 0:
        drone.send_movement_command_XYA(0, 0,flight_altitude)
    else:
        movementRollAngle = (pidRoll(inputValueVelocityX) * -1)
        drone.send_movement_command_XYA(movementRollAngle, 0,flight_altitude)
        debug_writer_ROLL(movementRollAngle)

def stop_drone():
    drone.send_movement_command_YAW(0)
    drone.send_movement_command_XYA(0, 0,flight_altitude)


interpreter = tflite.Interpreter(model_path="model.tflite") #change this to your model path
interpreter.allocate_tensors()


#get the input and output tensors so we can feed in values and get the results
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
input_shape = input_details[0]['shape']

#initialize the camera
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


def initialize_detector():
    global net, camera
    #initialize your object detection model here (ssd mobnet v2)
        

#connect to the Vehicle (in this case a UDP endpoint)
def connect_drone(connection_string,wait_ready=True, baud=57600):
    global vehicle
    if vehicle == None:
        vehicle = connect(connection_string, baud=baud, wait_ready=wait_ready)
        print ("drone connected")

#disconnect from the vehicle
def disconnect_drone():
    vehicle.close()

def get_version():
    global vehicle
    return vehicle.version

def get_mission():
    global vehicle
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    return cmds

def get_location():
    global vehicle
    return vehicle.location.global_frame

def get_altitude():
    global vehicle
    return vehicle.location.global_relative_frame.alt

def get_velocity():
    global vehicle
    return vehicle.velocity

def get_battery_info():
    global vehicle
    return vehicle.battery

def get_mode():
    global vehicle
    return vehicle.mode.name

def get_home_location():
    global vehicle
    return vehicle.home_location

def get_heading():
    global vehicle
    return vehicle.heading

def get_ekf_status():
    global vehicle
    return vehicle.ekf_ok

def get_groundspeed():
    global vehicle
    return vehicle.groundspeed

def read_channel(channel):
    return vehicle.channels[str(channel)]

def set_gimbal_angle(angle):
    global vehicle
    print("gimbal angle set to: " % angle)
    return vehicle.gimbal.rotate(0, angle, 0)

def set_groundspeed(speed):
    global vehicle
    print("groundspeed set to: " % speed)
    vehicle.groundspeed = speed

def set_flight_mode(f_mode):
    global vehicle
    vehicle.mode = VehicleMode(f_mode)

def set_param(param, value):
    global vehicle
    vehicle.parameters[param] = value

def get_param(param):
    return vehicle.parameters[param] 

def set_channel(channel, value):
    global vehicle
    vehicle.channels.overrides[channel] = value

def clear_channel(channel):
    global vehicle
    vehicle.channels.overrides[channel] = None
    
def get_channel_override(channel):
    return vehicle.channels.overrides[channel]
          
def disarm():
    global vehicle
    vehicle.armed = False

def arm():
    global vehicle
    vehicle.groundspeed = 3

    print ("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)

    print ("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("STABILIZE")
    vehicle.armed   = True

    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)

    print ("ARMED! Let's take OFF")

def arm_and_takeoff(aTargetAltitude):
    global vehicle

    #set default groundspeed low for safety 
    print ("setting groundspeed to 3")
    vehicle.groundspeed = 3

    print ("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)

    print ("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)

    print ("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)

def land():
    global vehicle
    print("Setting LAND mode...")
    vehicle.mode = VehicleMode("LAND")

def return_to_launch_location():
    #carefull with using this! It wont detect obstacles!
    vehicle.mode = VehicleMode("RTL")

def send_movement_command_YAW(heading):
    global vehicle
    speed = 0 
    direction = 1 #direction -1 ccw, 1 cw
    
    #heading 0 to 360 degree. if negative then ccw 
    
    print("Sending YAW movement command with heading: %f" % heading)

    if heading < 0:
        heading = heading*-1
        direction = -1

    #point drone into correct heading 
    msg = vehicle.message_factory.command_long_encode(
        0, 0,       
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
        0,          
        heading,    
        speed,      #speed deg/s
        direction,  
        1,          #relative offset 1
        0, 0, 0)    

    # send command to vehicle
    vehicle.send_mavlink(msg)
    #Vehicle.commands.flush()

def send_movement_command_XYA(velocity_x, velocity_y, altitude):
    global vehicle

    #velocity_x positive = forward. negative = backwards
    #velocity_y positive = right. negative = left
    #velocity_z positive = down. negative = up (Yes really!)

    print("Sending XYZ movement command with v_x(forward/backward): %f v_y(right/left): %f " % (velocity_x,velocity_y))

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,      
        0, 0,    
        mavutil.mavlink.MAV_FRAME_BODY_NED,  #relative to drone heading pos relative to EKF origin
        0b0000111111100011, #ignore velocity z and other pos arguments
        0, 0, altitude,
        velocity_x, velocity_y, 0, 
        0, 0, 0, 
        0, 0)    

    vehicle.send_mavlink(msg)
    #Vehicle.commands.flush()

def connect_lidar(serialString):
    global ser
    ser = serial.Serial(serialString, 115200, timeout=0)
    if ser.isOpen() == False:
        ser.open()
        return "success"
    else:
        return "port already open"
    
def disconnect_lidar():
        global ser
        ser.close()

def check_connection():
    global ser
    return ser.isOpen()

def read_lidar_distance():
    global ser
    while True:
        counter = ser.in_waiting
        if counter > 8:
            bytes_serial = ser.read(7) 
            ser.reset_input_buffer() 
            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59: 
                distance = bytes_serial[2] + bytes_serial[3]*256 
                strength = bytes_serial[4] + bytes_serial[5]*256 
                return distance/100.0,strength

def read_lidar_temperature():
    global ser
    while True:
        counter = ser.in_waiting
        if counter > 8:
            bytes_serial = ser.read(9) 
            ser.reset_input_buffer() 
           
            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59: 
                temperature = bytes_serial[6] + bytes_serial[7]*256 
                temperature = (temperature/8.0) - 256.0 
                return temperature



#zed 2 required variables 
depth_camera = None
runtime = None
depth_mat = None
rgb_mat = None
init_parameters = None
initialized = False

MAX_RANGE = 6
MIN_RANGE = 1

def set_params(performance_mode=True):
    global init_parameters
    init_parameters = sl.InitParameters()
    init_parameters.depth_mode = sl.DEPTH_MODE.PERFORMANCE if performance_mode else sl.DEPTH_MODE.ULTRA
    init_parameters.depth_minimum_distance = MIN_RANGE #1 Meter minimum detection distance
    init_parameters.coordinate_units = sl.UNIT.METER


def set_runtime_params():
    global runtime
    runtime = sl.RuntimeParameters()

def close():
    global depth_camera
    depth_camera.close()


def init_zed(performance_mode=True):
    global depth_mat, rgb_mat, depth_camera, initialized

    set_params(performance_mode)
    depth_camera = sl.Camera(init_parameters)

    if not depth_camera.is_opened():
        status = depth_camera.open()
        initialized = True
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))

    set_runtime_params()
    #depth_camera.set_depth_max_range_value(MAX_RANGE)
    depth_camera.open()

    depth_mat = sl.Mat()
    rgb_mat = sl.Mat()
    return depth_camera


def get_depth_image():
    global depth_mat
    image = None
    err = depth_camera.grab(runtime)
    if err == sl.ERROR_CODE.SUCCESS:
        depth_camera.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
        image = depth_mat.get_data()
    return image

def get_rgbd_image():
    global depth_mat, rgb_mat
    depth_image = None
    rgb_image = None
    err = depth_camera.grab(runtime)
    if err == sl.ERROR_CODE.SUCCESS:
        depth_camera.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
        depth_camera.retrieve_image(rgb_mat, sl.VIEW.LEFT)
        rgb_image = rgb_mat.get_data()
        depth_image = depth_mat.get_data()
        depth_image = cv2.normalize(depth_image,None,0,255,cv2.NORM_MINMAX,cv2.CV_8U)
        splitted = cv2.split(rgb_image)
        bgrd = cv2.merge((splitted[0],splitted[1],splitted[2],depth_image))
        return bgrd
    return None

        
if __name__ == "__main__":
    init_zed()
    while True:
        bgrd = get_rgbd_image()
        cv2.imshow("BGRD" , bgrd)
        cv2.waitKey(1)


def getCenter(contour):
    M = cv2.moments(contour)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return cx, cy

def getDelta(point1, point2):
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def get_single_axis_delta(value1, value2):
    return abs(value1 - value2)

def point_in_rectangle(point, left, right, top, bottom):
    if point[0] > left and point[0] < right and point[1] > top and point[1] < bottom:
        return True
    else:
        return False
    
def process(output_img):
    gray = cv2.cvtColor(output_img, cv2.COLOR_BGR2GRAY)
    contours, hierachy = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    centerpoint = (round(output_img.shape[1]/2), round(output_img.shape[0]/2))

    filtered_contours = []
    for contour in contours:
        area = cv2.contourArea(contour)
        print(area)
        if area > 100000.0:
            filtered_contours.append(contour)
            
    count = 0
    target = ((centerpoint[0], centerpoint[1]), 0)
    if len(filtered_contours)>1:
        #decision needs to be made to which target drone needs to go 
        for contours in filtered_contours:
            targetCenter = getCenter(contours)
            if count == 0:
                targetCenter = getCenter(contours)
                delta = getDelta(centerpoint, targetCenter)
                target = (targetCenter, delta)
            else:
                currentDelta = getDelta(centerpoint, targetCenter)
                if abs(target[1] - currentDelta):
                    targetCenter = getCenter(contours)
                    target = (targetCenter, currentDelta)
            count += 1


    elif len(filtered_contours) == 1:
        targetCenter = getCenter(filtered_contours[0])
        delta = getDelta(centerpoint, targetCenter)
        target = (targetCenter, delta)

    else:
        cv2.putText(output_img, "No Target Detected", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        #show target 
    cv2.circle(output_img, target[0], 20, (0, 0, 255), thickness=-1, lineType=8, shift=0)
    cv2.putText(output_img, str(target[1]), (50, 50), cv2.FONT_HERSHEY_SIMPLEX , 1, (0, 0, 255), 3, cv2.LINE_AA) 
    #show path
    cv2.line(output_img, centerpoint, target[0], (255, 0, 0), thickness=10, lineType=8, shift=0)

    #show center
    cv2.circle(output_img, centerpoint, 20, (0, 255, 0), thickness=-1, lineType=8, shift=0)

    #show contours
    cv2.drawContours(output_img, filtered_contours, -1, (255, 255, 255), 3)

    return output_img

#main loop
while True:
    ret, frame = camera.read()

    # Preprocess the frame (resize, normalize, etc.) according to the model's requirements
    input_data = cv2.resize(frame, (input_shape[1], input_shape[2]))
    input_data = np.expand_dims(input_data, axis=0)
    input_data = (input_data.astype(np.float32) - 127.5) / 127.5  # Normalize to [-1, 1]

    # Set the input tensor to the model
    interpreter.set_tensor(input_details[0]['index'], input_data)

    # Run inference
    interpreter.invoke()

    # Get the output tensor
    output_data = interpreter.get_tensor(output_details[0]['index'])

    # Process the output_data (e.g., draw bounding boxes) based on your application

    # Display the frame with detections
    cv2.imshow("SSD MobileNet V2 Detection", frame)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
camera.release()
cv2.destroyAllWindows()

# Function to transform local coordinates to camera frame
def transform_to_camera_frame(tVec, rVec, local_coordinates):
    rotation_matrix, _ = cv2.Rodrigues(rVec)
    aruco_camera_frame = np.dot(rotation_matrix, local_coordinates.T).T + tVec
    return aruco_camera_frame

# Load camera calibration parameters (intrinsic and distortion)
calib_data_path = "/home/raspberrypi/calibration.npz"
calib_data = np.load(calib_data_path)
mtx, dist = calib_data["mtx"], calib_data["dist"]

#define the pid values
Kp = 0.1
Ki = 0.1
Kd = 0.1

#initialize the integral and previous error
integral = 0
previous_error = 0


#function for pid logic 
def pid(error, previous_error, integral, Kp, Ki, Kd):
    integral = integral + error
    derivative = error - previous_error
    output = Kp * error + Ki * integral + Kd * derivative
    return output, integral

#pid control for pitch and roll
def pid_control_pitch_roll(error, previous_error, integral, Kp, Ki, Kd):
    output, integral = pid(error, previous_error, integral, Kp, Ki, Kd)
    return output, integral

#pid control for yaw
def pid_control_yaw(error, previous_error, integral, Kp, Ki, Kd):
    output, integral = pid(error, previous_error, integral, Kp, Ki, Kd)
    return output, integral

#function to calculate pitch angle
def calculate_pitch_angle(rVec):
    rotation_matrix, _ = cv2.Rodrigues(rVec)
    pitch_angle = np.arctan2(-rotation_matrix[2, 0], np.sqrt(rotation_matrix[2, 1]**2 + rotation_matrix[2, 2]**2))
    return pitch_angle

#function to calculate roll angle
def calculate_roll_angle(rVec):
    rotation_matrix, _ = cv2.Rodrigues(rVec)
    roll_angle = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
    return roll_angle

#function for vehicle to pitch
def vehicle_pitch(pitch_angle):
    msg = vehicle.message_factory.command_long_encode(
    0, 0,  # target system, target component
    mavutil.mavlink.MAV_CMD_CONDITION_PITCH_ROLL,  # command
    0,  # confirmation
    pitch_angle,  # param 1, pitch in degrees
    0,  # param 2, roll in degrees
    0,  # param 3, yaw in degrees
    0,  # param 4, thrust
    0, 0, 0)  # param 5 ~ 7 not used
    vehicle.send_mavlink(msg)

#function for vehicle to roll
def vehicle_roll(roll_angle):
    msg = vehicle.message_factory.command_long_encode(
    0, 0,  # target system, target component
    mavutil.mavlink.MAV_CMD_CONDITION_PITCH_ROLL,  # command
    0,  # confirmation
    0,  # param 1, pitch in degrees
    roll_angle,  # param 2, roll in degrees
    0,  # param 3, yaw in degrees
    0,  # param 4, thrust
    0, 0, 0)  # param 5 ~ 7 not used
    vehicle.send_mavlink(msg)

#function for vehicle to yaw
def vehicle_yaw(yaw_angle):
    msg = vehicle.message_factory.command_long_encode(
    0, 0,  # target system, target component
    mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
    0,  # confirmation
    yaw_angle,  # param 1, yaw in degrees
    0,  # param 2, yaw speed deg/s
    1,  # param 3, direction -1 ccw, 1 cw
    0,  # param 4, relative offset 1, absolute angle 0
    0, 0, 0)  # param 5 ~ 7 not used
    vehicle.send_mavlink(msg)



#function to calculate yaw angle
def calculate_yaw_angle(rVec):
    rotation_matrix, _ = cv2.Rodrigues(rVec)
    yaw_angle = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    return yaw_angle


#function for vehicle to yaw 
def vehicle_yaw(yaw_angle):
    msg = vehicle.message_factory.command_long_encode(
    0, 0,  # target system, target component
    mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
    0,  # confirmation
    yaw_angle,  # param 1, yaw in degrees
    0,  # param 2, yaw speed deg/s
    1,  # param 3, direction -1 ccw, 1 cw
    0,  # param 4, relative offset 1, absolute angle 0
    0, 0, 0)  # param 5 ~ 7 not used
    vehicle.send_mavlink(msg)


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

MARKER_SIZE = 9.3  # centimeters

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

#pid controls 
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        self.integral += error
        derivative = error - self.prev_error

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        self.prev_error = error
        return output

# Example usage
pid_controller = PIDController(Kp=0.1, Ki=0.01, Kd=0.05)

while True:
    # Get current position and desired position (from ArUco marker)
    current_position = get_current_position()
    desired_position = get_desired_position()

    # Compute error
    error = desired_position - current_position

    # Compute PID output
    pid_output = pid_controller.compute(error)

    # Convert PID output to velocity commands
    velocity_x, velocity_y = convert_pid_output_to_velocity(pid_output)

    # Apply velocity commands to control drone's movement
    move_drone(velocity_x, velocity_y)

    # Continue the loop
    time.sleep(0.1)





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

desired_loop_frequency = 20  # Set your desired loop frequency in Hz

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


                #doing pitch based on calculation
                vehicle_pitch(calculate_pitch_angle(rVec[i]))

                #doing roll based on calculation
                vehicle_roll(calculate_roll_angle(rVec[i]))


            fb = 0
            if distance < 200:             
#                fb = (distance - 200)/100 #50
                fb = -1.0
                print(fb)

                #doing yaw based on calculation 
                vehicle_yaw(calculate_yaw_angle(rVec[i]))


                print("Aruco marker is very close ,vehicle is moving backward to ", aruco_lat, aruco_lon, aruco_alt)
                
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
                print("Aruco marker is in correct position, vehicle will remain stationary")
                fb= 0
                print(fb)
                
                #doing yaw based on calculation
                vehicle_yaw(calculate_yaw_angle(rVec[i]))
                
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
#                fb = (distance - 210)/100
                fb = 1.0
                #print(fb)
                print("Aruco marker is very far ,vehicle is moving forward to ", aruco_lat, aruco_lon, aruco_alt)
                
                #doing yaw based on calculation
                vehicle_yaw(calculate_yaw_angle(rVec[i]))

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

