from raspConfig import *

picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 30

picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

sleep(2)

ser.reset_input_buffer()
ser.reset_output_buffer()

ser.write("$C\n".encode())
while True:
    line = ser.readline().decode().strip() if ser.in_waiting > 0 else None
    if line == "Ready!":
        break

send_servo(SERVO_CENTER)
sleep(1)
send_motor(MOTOR_DRIVE)

try:
    while True:
        frame = picam2.capture_array()

        now = time.monotonic()

        redArea, redMask, redContour = detect_pillars(frame, ROI_PILLARS, 
                                                    HSV_RED_LOWER1, HSV_RED_UPPER1, 
                                                    HSV_RED_LOWER2, HSV_RED_UPPER2)
        greenArea, greenMask, greenContour = detect_pillars(frame,ROI_PILLARS, 
                                                            HSV_GREEN_LOWER, 
                                                            HSV_GREEN_UPPER)

        if redArea > greenArea:
            mainPillar = redContour
            mainColor = "RED"
        elif greenArea > redArea:
            mainPillar = greenContour
            mainColor = "GREEN"
        else:
            mainPillar = None
            mainColor = None

        leftContour, leftMask, leftArea = find_wall_area(frame, ROI_LEFT)
        rightContour, rightMask, rightArea = find_wall_area(frame, ROI_RIGHT)
        if mainPillar is not None:
            if mainColor == "RED":
                error = (leftArea + redArea*3) - rightArea
            else:
                error = leftArea - (rightArea + greenArea*3)
        else:
            error = leftArea - rightArea
        
        lineDetected, centerMask, clockwise = detect_colored_line(frame)


        if lineDetected and not linePresent:

            lineCount += 1

            linePresent = True

            if lineCount == 1:
                goClockwise = clockwise
                print(f"turn right/clockwise: {goClockwise}")

        elif not lineDetected:

            linePresent = False

        if abs(followEnterTime - now) >= FOLLOW_TIME_THRESH and not useIMU:
            enterHeading = grab_heading()
            print(f"Enter Heading: {enterHeading}")
            useIMU = True

        if useIMU:
            currentHeading = grab_heading()
            if abs(currentHeading - enterHeading) >= EXIT_ANGLE_THRESH:
                print(f"Exit Heading: {currentHeading}")
                print(f"Turns Done: {turnsCompleted}")
                turnsCompleted += 1
                currentHeading = 0
                enterHeading = 0
                followEnterTime = now
                passedAngle = True
                useIMU = False
            else:
                passedAngle = False

        if turnsCompleted == 12 and not stopScheduled:
            stopScheduled = True
            stopTime = now
            print("12 turns done\nStopping in 2 seconds")
            
        if stopScheduled:

            if now - stopTime >= STOP_DELAY:

                send_motor(MOTOR_STOP)
                send_servo(SERVO_CENTER)

                print("Robot stopped")

                break

        if mode == "FOLLOW_WALL":

            error /= roiArea 
            error *= SERVO_RANGE

            steering = pd_controller(error)

            send_servo(steering)

            activate_led(LEDB)
            
            tooMuchWall = leftArea > ENTER_TURN_THRESH_UPPER and rightArea > ENTER_TURN_THRESH_UPPER
            normalTurn = leftArea < ENTER_TURN_THRESH_LOWER or rightArea < ENTER_TURN_THRESH_LOWER

            if normalTurn or tooMuchWall:
                confirmCount += 1
            else:
                confirmCount = 0

            '''if confirmCount >= CONFIRM_FRAMES:

                if linePresent:
                    if leftArea < rightArea:
                        turnSide = "LEFT"
                        send_servo(SERVO_LEFT)

                    elif rightArea < leftArea:
                        turnSide = "RIGHT"
                        send_servo(SERVO_RIGHT)
                    mode = "TURNING"
                    turnEnterTime = now
                    confirmCount = 0
                    activate_led(LEDG)
                    
                elif tooMuchWall:
                    if goClockwise:
                        turnSide = "RIGHT"
                        send_servo(SERVO_RIGHT)
                    else:
                        turnSide = "LEFT"
                        send_servo(SERVO_LEFT)
                    mode = "TURNING"
                    turnEnterTime = now
                    confirmCount = 0
                    activate_led(LEDG)

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
                if passedAngle:
                    mode = "FOLLOW_WALL"
                    turnSide = None
                    turnEnterTime = None
                    prevError = 0'''

        draw_roi(frame, ROI_LEFT, "LEFT")
        draw_roi(frame, ROI_RIGHT, "RIGHT")
        draw_roi(frame, ROI_CENTER, "CENTER")
        draw_roi(frame, ROI_PILLARS, "PILLARS")

        if leftContour is not None:
            cv2.drawContours(frame, [leftContour], -1, (0, 255, 255), 2)

        if rightContour is not None:
            cv2.drawContours(frame, [rightContour], -1, (0, 255, 255), 2)

        if mainPillar is not None:
            x, y, w, h = cv2.boundingRect(mainPillar)
            cx = x + w // 2 # horizontal center of pillar, relative to crop
            cy = y + h // 2 # vertical center of pillar, relative to crop

            # Convert to full-frame coordinates for display
            cx_full = cx + ROI_PILLARS[0]
            cy_full = cy + ROI_PILLARS[1]

            cv2.rectangle(frame, (x + ROI_PILLARS[0], y + ROI_PILLARS[1]), 
                        (x + w + ROI_PILLARS[0], y + h + ROI_PILLARS[1]), 
                        (255, 255, 0), 2)

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

        cv2.putText(
            frame,
            f"Turns Done: {turnsCompleted}",
            (200, 130),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )
        
        cv2.putText(
			frame,
			f"Confirm Count: {confirmCount}",
			(250, 30),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.7,
			(255, 255, 0),
			2
		)

        cv2.imshow("Frame", frame)
        cv2.imshow("Center Mask", centerMask)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except Exception as e:
    traceback.print_exc()

finally:
    send_motor(MOTOR_STOP)
    sleep(1)
    send_servo(SERVO_CENTER)
    cv2.destroyAllWindows()
    picam2.stop()
    ser.close()
