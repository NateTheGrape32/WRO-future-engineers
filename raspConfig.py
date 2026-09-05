import cv2
import serial
import traceback
import time
from time import sleep
from picamera2 import Picamera2
import numpy as np

SERIAL_PORT = "/dev/ttyACM0"
BAUD = 115200

SERVO_CENTER = 80
SERVO_LEFT = 65
SERVO_RIGHT = 95
SERVO_RANGE = 85

MOTOR_STOP = 1500
MOTOR_DRIVE = 1600

LEDB = 22
LEDR = 23
LEDG = 24

ROI_LEFT = [30, 280, 180, 610]
ROI_RIGHT = [490, 280, 640, 610]
ROI_CENTER = [180, 375, 490, 415]
ROI_PILLARS = [0, 200, 640, 480]

LAB_LOWER = np.array([0, 0, 0], dtype=np.uint8)
LAB_UPPER = np.array([65, 255, 255], dtype=np.uint8)

HSV_RED_LOWER1 = np.array([0, 90, 90])
HSV_RED_UPPER1 = np.array([7, 255, 255])
HSV_RED_LOWER2 = np.array([171, 80, 80])
HSV_RED_UPPER2 = np.array([179, 255, 255])

HSV_GREEN_LOWER = np.array([40, 100, 0])
HSV_GREEN_UPPER = np.array([85, 255, 255])

PILLAR_THRESH = 100

ENTER_TURN_THRESH_LOWER = 300
ENTER_TURN_THRESH_UPPER = 20000
EXIT_TURN_THRESH = 4000

CONFIRM_FRAMES = 5
EXIT_TIME_THRESH = 0.75
EXIT_ANGLE_THRESH = 80

FOLLOW_TIME_THRESH = 3

BLUE_LOWER = np.array([95,80,80])
BLUE_UPPER = np.array([130,255,255])

ORANGE_LOWER = np.array([5,100,100])
ORANGE_UPPER = np.array([25,255,255])

LINE_THRESH = 1000
STOP_DELAY = 2.0

prevError = 0
mode = "FOLLOW_WALL"
turnSide = None
confirmCount = 0
turnEnterTime = None
turnsCompleted = 0
enterHeading = 0
currentHeading = 0
passedAngle = False

followEnterTime = 0
useIMU = False

mainPillar = None
mainColor = None

lineCount = 0
linePresent = False
goClockwise = False

stopScheduled = False
stopTime = None

roiArea = (ROI_LEFT[2] - ROI_LEFT[0]) * (ROI_LEFT[3] - ROI_LEFT[1])

ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)

def clamp(value, low=25, high=130):
    return max(low, min(high, value))

def send_servo(angle):
    angle = int(clamp(angle))
    ser.write(f"$S{angle}\n".encode())

def send_motor(speed):
    ser.write(f"$M{int(speed)}\n".encode())
    
def activate_led(led):
	ser.write(f"$L{led}\n".encode())

def grab_heading():
    ser.reset_input_buffer()
    sleep(0.1)
    ser.write("$I\n".encode())
    while True:
        if ser.in_waiting > 0:
            heading = float(ser.readline().decode().strip())
            break
    return heading

def draw_roi(img, roi, label = ""):
    x1, y1, x2, y2 = roi
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

    if label:
        cv2.putText(
            img,
            label,
            (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )


def find_wall_area(frame, roi):
    x1, y1, x2, y2 = roi


    roiImg = frame[y1:y2, x1:x2]

    lab = cv2.cvtColor(roiImg, cv2.COLOR_RGB2Lab)
    lab = cv2.GaussianBlur(lab, (7, 7), 0)

    mask = cv2.inRange(lab, LAB_LOWER, LAB_UPPER)
    _, maskRed, __ = detect_pillars(frame, roi, HSV_RED_LOWER1, HSV_RED_UPPER1, HSV_RED_LOWER2, HSV_RED_UPPER2)
    _, maskGreen, __ = detect_pillars(frame, roi, HSV_GREEN_LOWER, HSV_GREEN_UPPER)
    
    mask = cv2.bitwise_xor(mask, maskRed)
    mask = cv2.bitwise_xor(mask, maskGreen)

    kernel = np.ones((5, 5), np.uint8)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    largest = None
    largestArea = 0

    for c in contours:
        area = cv2.contourArea(c)

        if area > largestArea:
            largestArea = area
            largest = c

    if largest is not None:
        largest += np.array([x1, y1])

    return largest, mask, largestArea

def detect_colored_line(frame):

    x1, y1, x2, y2 = ROI_CENTER

    roi = frame[y1:y2, x1:x2]

    hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)

    blueMask = cv2.inRange(
        hsv,
        BLUE_LOWER,
        BLUE_UPPER
    )

    orangeMask = cv2.inRange(
        hsv,
        ORANGE_LOWER,
        ORANGE_UPPER
    )

    kernel = np.ones((3,3), np.uint8)

    blueMask = cv2.morphologyEx(
        blueMask,
        cv2.MORPH_OPEN,
        kernel
    )
    
    orangeMask = cv2.morphologyEx(
		orangeMask,
		cv2.MORPH_OPEN,
		kernel
    )
    
    mask = cv2.bitwise_or(blueMask, orangeMask)


    areaBlue = cv2.countNonZero(blueMask)
    areaOrange = cv2.countNonZero(orangeMask)


    detected = areaBlue > LINE_THRESH or areaOrange > LINE_THRESH
    
    clockwise = True if areaOrange else False

    return detected, mask, clockwise


def detect_pillars(frame, roi, hsv_lower1, hsv_upper1, hsv_lower2=None, hsv_upper2=None):
    x1, y1, x2, y2 = roi

    pillarCrop = frame[y1:y2, x1:x2]

    hsv = cv2.cvtColor(pillarCrop, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (7, 7), 0)

    mask1 = cv2.inRange(hsv, hsv_lower1, hsv_upper1)
    if hsv_lower2 is not None and hsv_upper2 is not None:
        mask2 = cv2.inRange(hsv, hsv_lower2, hsv_upper2)
        mask = cv2.bitwise_or(mask1, mask2)

    else:
        mask = mask1

    kernel = np.ones((5, 5), np.uint8)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    maxArea = 0
    largest = None

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        area = cv2.contourArea(c)
        if area > maxArea and area > PILLAR_THRESH:
            maxArea = area
            largest = c

    return maxArea, mask, largest

def pd_controller(error, kp=0.7, kd=0.3):
    global prevError

    derivative = error - prevError
    output = kp * error + kd * derivative

    prevError = error

    steering = SERVO_CENTER + output
    steering = clamp(steering)
    

    return steering