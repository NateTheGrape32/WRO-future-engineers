import cv2
import numpy as np
import serial
import time
from time import sleep
from picamera2 import Picamera2

SERIAL_PORT = "/dev/ttyACM0"
BAUD = 115200

SERVO_CENTER = 80
SERVO_LEFT = 65
SERVO_RIGHT = 95
SERVO_RANGE = 85

MOTOR_STOP = 1500
MOTOR_DRIVE = 1600

ROI_LEFT = [30, 280, 180, 610]
ROI_RIGHT = [490, 280, 640, 610]
ROI_CENTER = [180, 375, 490, 415]

LAB_LOWER = np.array([0, 0, 0], dtype=np.uint8)
LAB_UPPER = np.array([80, 255, 255], dtype=np.uint8)

ENTER_TURN_THRESH = 300
EXIT_TURN_THRESH = 4000

CONFIRM_FRAMES = 5
EXIT_TIME_THRESH = 0.75

EXIT_ANGLE_THRESH = 80

BLUE_LOWER = np.array([95,80,80])
BLUE_UPPER = np.array([130,255,255])

ORANGE_LOWER = np.array([5,100,100])
ORANGE_UPPER = np.array([25,255,255])

LINE_THRESH = 1000
STOP_DELAY = 5.0

picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 30

picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

sleep(2)

ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
ser.reset_input_buffer()
ser.reset_output_buffer()

ser.write("$C\n".encode())
while True:
    line = ser.readline().decode().strip() if ser.in_waiting > 0 else None
    if line == "Zeroed. Go!":
        break

prevError = 0
mode = "FOLLOW_WALL"
turnSide = None
confirmCount = 0
turnEnterTime = None
turnsCompleted = 0
enterHeading = 0
currentHeading = 0

lineCount = 0
linePresent = False

stopScheduled = False
stopTime = None

roiArea = (ROI_LEFT[2] - ROI_LEFT[0]) * (ROI_LEFT[3] - ROI_LEFT[1])

def clamp(value, low=25, high=130):
    return max(low, min(high, value))

def send_servo(angle):
    angle = int(clamp(angle))
    ser.write(f"$S{angle}\n".encode())

def send_motor(speed):
    ser.write(f"$M{int(speed)}\n".encode())
    
def activate_led(led):
	ser.write(f"$L{led}\n".encode())

def grab_heading(heading):
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


    mask = cv2.bitwise_or(
        blueMask,
        orangeMask
    )


    kernel = np.ones((3,3), np.uint8)

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel
    )


    area = cv2.countNonZero(mask)


    detected = area > LINE_THRESH


    return detected, mask

def pd_controller(error, kp=0.7, kd=0.3):
    global prevError

    derivative = error - prevError
    output = kp * error + kd * derivative

    prevError = error

    steering = SERVO_CENTER + output
    steering = clamp(steering)
    

    return steering


send_servo(SERVO_CENTER)
sleep(1)
send_motor(MOTOR_DRIVE)

while True:


    frame = picam2.capture_array()

    now = time.monotonic()

    leftContour, leftMask, leftArea = find_wall_area(frame, ROI_LEFT)
    rightContour, rightMask, rightArea = find_wall_area(frame, ROI_RIGHT)
    
    lineDetected, centerMask = detect_colored_line(frame)


    if lineDetected and not linePresent:

        lineCount += 1

        print(f"Lines detected: {lineCount}")

        linePresent = True


        if lineCount == 23:

            stopScheduled = True
            stopTime = now

            print("23rd line detected")
            print("Stopping in 3 seconds")


    elif not lineDetected:

        linePresent = False
        
    if stopScheduled:

        if now - stopTime >= STOP_DELAY:

            send_motor(MOTOR_STOP)
            send_servo(SERVO_CENTER)

            print("Robot stopped")

            break

    if mode == "FOLLOW_WALL":

        error = (leftArea - rightArea) / roiArea * SERVO_RANGE

        steering = pd_controller(error)

        send_servo(steering)

        activate_led(22)

        if leftArea < ENTER_TURN_THRESH or rightArea < ENTER_TURN_THRESH:
            confirmCount += 1
        else:
            confirmCount = 0

        if confirmCount >= CONFIRM_FRAMES and linePresent:

            if leftArea < rightArea:
                turnSide = "LEFT"
                send_servo(SERVO_LEFT)

            elif rightArea < leftArea:
                turnSide = "RIGHT"
                send_servo(SERVO_RIGHT)

            mode = "TURNING"
            turnEnterTime = now
            confirmCount = 0
            ser.reset_input_buffer()
            sleep(0.1)
            ser.write("$I\n".encode())
            if ser.in_waiting > 0:
                enterHeading = float(ser.readline().decode().strip())
            print(enterHeading)
            activate_led(23)

    elif mode == "TURNING":

        elapsed = now - turnEnterTime

        wallSeenAgain = (
            (turnSide == "LEFT" and leftArea > EXIT_TURN_THRESH)
            or
            (turnSide == "RIGHT" and rightArea > EXIT_TURN_THRESH)
            or
            (
                turnSide == "BOTH"
                and leftArea > EXIT_TURN_THRESH
                and rightArea > EXIT_TURN_THRESH
            )
        )

        if elapsed > EXIT_TIME_THRESH and wallSeenAgain:
            ser.write("$I\n".encode())
            if ser.in_waiting > 0:
                ser.reset_input_buffer()
                sleep(0.1)
                currentHeading = float(ser.readline().decode().strip())
                print(currentHeading)
            if abs(currentHeading - enterHeading) >= EXIT_ANGLE_THRESH:
                mode = "FOLLOW_WALL"
                turnSide = None
                turnEnterTime = None
                prevError = 0
                turnsCompleted += 1
                currentHeading = 0
                enterHeading = 0

    draw_roi(frame, ROI_LEFT, "LEFT")
    draw_roi(frame, ROI_RIGHT, "RIGHT")
    draw_roi(frame, ROI_CENTER, "CENTER")

    if leftContour is not None:
        cv2.drawContours(frame, [leftContour], -1, (0, 255, 255), 2)

    if rightContour is not None:
        cv2.drawContours(frame, [rightContour], -1, (0, 255, 255), 2)

    cv2.putText(
        frame,
        f"Mode: {mode}",
        (20, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 0, 255),
        2
    )
    
    cv2.putText(
        frame, 
        f"Left Area: {leftArea}, Right Area: {rightArea}", 
        (20, 80), 
        cv2.FONT_HERSHEY_SIMPLEX, 
        0.7, 
        (255, 0, 0), 
        2
    )
    
    cv2.putText(
		frame,
		f"Line Count: {lineCount}",
		(20, 130),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.7,
		(0, 255, 0),
		2
	)

    cv2.imshow("Frame", frame)
    cv2.imshow("Center Mask", centerMask)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break


send_motor(MOTOR_STOP)
sleep(1)
send_servo(SERVO_CENTER)
cv2.destroyAllWindows()
picam2.stop()
ser.close()
