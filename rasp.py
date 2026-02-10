import cv2, numpy as np
from time import sleep
from picamera2 import Picamera2
import time, serial

# --- Camera Init --- #
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 30
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()
sleep(2)  # Allow camera to warm up and arduino port to boot

ser = serial.Serial('/dev/ttyACM0', 112500, timeout=1)  # Update with your Arduino's port
ser.reset_input_buffer()  # Clear any pending input/output
ser.reset_output_buffer()

# --- ROIs (x1, y1, x2, y2) --- #
roi1 = [20, 170, 240, 220] # Left side
roi2 = [400, 170, 620, 220] # Right side

def drawRoi(img, roi, color=(0, 255, 0), thickness=2, label=None):
    #Draws a rectangle on the image to indicate the ROI.
    x1, y1, x2, y2 = roi
    cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)
    if label:
        cv2.putText(img, label, (x1, max(15, y1 - 8)), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)
    return img

def findWallAreaLab(frameRgb, roi, labLower, labUpper, minContourArea=50):
    """
    Returns:
      areaMax: area of largest contour inside ROI (0 if none)
      contourMax: contour points (shifted to full-frame coordinates) or None
      mask: binary mask for ROI (for optional debug display)
    """
    #Calculates the percentage of pixels within the specified LAB color range in the ROI.
    x1, y1, x2, y2 = roi
    roiImg = frameRgb[y1:y2, x1:x2] # picamera2 gives RGB, but OpenCV ops below work either way for Lab conversion
    roiLab = cv2.cvtColor(roiImg, cv2.COLOR_RGB2Lab)
    roiLab = cv2.GaussianBlur(roiLab, (7, 7), 0) # larger kernel size means more smoothing
    mask = cv2.inRange(roiLab, labLower, labUpper)

    # Clean up noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    areaMax = 0
    contourMax = None
    for c in contours:
        area = cv2.contourArea(c)
        if area >= minContourArea and area > areaMax:
            areaMax = area
            contourMax = c
    if contourMax is not None:
        contourMax += np.array([x1, y1], dtype=np.uint32)  # Shift contour to full-frame coordinates for drawing on the OG frame
    return contourMax, mask, areaMax, #wallAreaPercent

def pdController(error, kp=0.1, kd=0.05):
    global prevError
    correction = (kp * error) + (kd * (error - prevError))
    prevError = error
    ser.write(f"$S{correction}\n")

def turn(side, speed=0.5):
    pass

# --- Lab threshold for black wall (tune if needed) --- #
LAB_LOWER = np.array([0, 0, 0], dtype=np.uint8)
LAB_UPPER = np.array([70, 255, 255], dtype=np.uint8)

# --- Turning state reqs --- #
ENTER_TURN_THRESH = 550  # area threshold to enter turning zone
EXIT_TURN_THRESH = 1200   # area threshold to exit turning zone
EXIT_TIME_THRESH = 10.0
EXIT_ANGLE_THRESH = 90.0 # minimum turn angle before allowing exit (prevents false exits from small turns or noise)

# --- False trigger prevention --- #
CONFIRM_FRAMES = 5  # number of consecutive frames to confirm turn
side = None
confirmCount = 0
turnEnterTime = None

# --- Turn degree variables --- #
enterTurnDegree = None
turnDegrees = None

prevError = 0

mode = "FOLLOW_WALL"

# --- Main Loop --- #
while True:
    frame = picam2.capture_array()

    now = time.monotonic()

    # Detect ROIs
    leftContour, leftMask, leftArea = findWallAreaLab(frame, roi1, LAB_LOWER, LAB_UPPER)
    rightContour, rightMask, rightArea = findWallAreaLab(frame, roi2, LAB_LOWER, LAB_UPPER)

    # --- Driving States --- #

    # --- Wall Follow --- #
    if mode == "FOLLOW_WALL":
        pdController(leftArea-rightArea)
        if leftArea < ENTER_TURN_THRESH or rightArea < ENTER_TURN_THRESH:
            confirmCount += 1
        else:
            confirmCount = 0
        
        if confirmCount >= CONFIRM_FRAMES:
            if leftArea < rightArea:
                side = "left"
            elif rightArea < leftArea:
                side = "right"
            else:
                side = "both"
            confirmCount = 0
            mode = "TURNING"
            turnEnterTime = now
            enterTurnAngle = int(ser.readline().decode().strip())
    
    # --- Turning --- #
    else:
        ser.write(f"$T{side}\n") # send turn command to arduino
        delta = abs(enterTurnDegree - int(ser.readline().decode().strip()))
        turnDegrees = min(delta, 360 - delta) # handle wraparound from 0 to 360
        elapsed = now - (turnEnterTime if turnEnterTime else now)
        if elapsed >= EXIT_TIME_THRESH and turnDegrees > EXIT_ANGLE_THRESH: # minimum time & turn angle before exit
            if (side == "both" and leftArea > EXIT_TURN_THRESH and rightArea > EXIT_TURN_THRESH)\
                or (side == "left" and leftArea > EXIT_TURN_THRESH)\
                or (side == "right" and rightArea > EXIT_TURN_THRESH): # check if wall seen again
                mode = "FOLLOW_WALL"
                side = None
                turnEnterTime = None
                enterTurnDegree = None
                turnDegrees = None

    # --- Visualization --- #
    # Draw ROIs and contours
    drawRoi(frame, roi1, label="Left ROI")
    drawRoi(frame, roi2, label="Right ROI")

    # Draw contours if found
    if leftContour is not None:
        cv2.drawContours(frame, [leftContour], -1, (0, 255, 255), 2)
    if rightContour is not None:
        cv2.drawContours(frame, [rightContour], -1, (0, 255, 255), 2)

    # show numeric values
    cv2.putText(frame, f"Left Area: {leftArea}", (roi1[0], roi1[1] - 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(frame, f"Right Area: {rightArea}", (roi2[0], roi2[1] - 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(frame, f"Confirm Count: {confirmCount}", (roi1[0], 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.putText(frame, f"Elapsed Time Since Turn Enter: {now - (turnEnterTime if turnEnterTime else now):.2f}s", (roi1[0], roi1[3]+30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # show mode
    cv2.putText(frame, f"Mode: {mode}", (roi1[0], 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.putText(frame, f"Turn Side: {side}", (roi1[0], 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # Display results
    cv2.imshow("Frame", frame)
    cv2.imshow("Mask ROI 1", leftMask)
    cv2.imshow("Mask ROI 2", rightMask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cv2.destroyAllWindows()
picam2.stop()