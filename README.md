# WRO-future-engineers
WRO 2026 Andrew and Nathan
YouTube Video: [Open Challenge](https://youtu.be/nm6dfaRhhao) 

# Materials
Originally we planned to use the following RC car for its chassis: [Original Chassis](https://www.amazon.ca/Full-Scale-Rear-Wheel-Rechargeable-Control-Crawler/dp/B0G24MWNGD?th=1)

However, this did not have the things that we wanted so we switched chassis to this one: [New Chassis](https://www.aliexpress.com/item/1005007171270606.html?spm=a2g0o.productlist.main.1.289cSZLuSZLuyz&algo_pvid=d5597024-2cfb-4d7d-91cb-67a7236b9c16&algo_exp_id=d5597024-2cfb-4d7d-91cb-67a7236b9c16-0&pdp_ext_f=%7B%22order%22%3A%22817%22%2C%22eval%22%3A%221%22%2C%22orig_sl_item_id%22%3A%221005007171270606%22%2C%22orig_item_id%22%3A%221005005831438904%22%2C%22fromPage%22%3A%22search%22%7D&pdp_npi=6%40dis%21CAD%21144.00%2167.68%21%21%21685.39%21322.13%21%4021032e4e17886469147393288e0f4b%2112000039693550543%21sea%21CA%210%21ABX%211%210%21n_tag%3A-29910%3Bd%3A61c648%3Bm03_new_user%3A-29895&curPageLogUid=kDYrA4otmimK&utparam-url=scene%3Asearch%7Cquery_from%3A%7Cx_object_id%3A1005007171270606%7C_p_origin_prod%3A1005005831438904)

In addition, we used this 2D LiDAR: [2D Lidar](https://www.aliexpress.com/item/1005003012681021.html?spm=a2g0o.order_list.order_list_main.31.67dd18029q37Y0)

This Pi camera: [Pi Camera](https://www.aliexpress.com/item/1005009258177201.htmlpdp_npi=6%40dis%21USD%21%2430.99%21%2415.19%21%21%21%21%21%402101e83017699792506122817ed50e%2112000048516190389%21btfpre%21%21%21%211%210%21&afTraceInfo=1005009258177201__pc__c_ppc_item_bridge_pc_related_wf__q8i25HV__1769979250883) 

This Pi Power Regulator: [Power Regulator](https://ca.robotshop.com/products/yahboom-power-supply-expansion-board-raspberry-pi-5) 

This Battery/Power Supply: https://genstattu.com/gens-ace-1300mah-2s-7-4v-45c-g-tech-lipo-battery-pack-with-deans-plug/?srsltid=AfmBOoo-qPXzcxuH2dIqTfVYg5ghG9WdKi2b53X-R9M8j3XF_JQlLKJL 

This Arduino Nano: [Arduino Nano](https://store-usa.arduino.cc/products/nano-33-ble-sense-rev2) 

This Button: [Button](https://www.amazon.ca/dp/B01J9KO7DC?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1)

Servo motor for driving: [Servo Motor](https://hitecrcd.com/hs-5055mg-economy-metal-gear-feather-servo/) 



# Mobility & Mechanical Design
In creating the chassis, we needed to incorporate these things which was done by:
- low center of gravity to maintain balance and reach higher speeds
    - creating a compact design as close to the ground as possible
    - Only light elements at the very top of the robot(camera)
- front wheel steering to have simple control over robot movement
    - choosing a chassis with Ackerman steering
    - controlling it with a servo motor
- differential gears
    - choosing a chassis with 2WD & differential gears
    - controlling it with a DC motor
- decent steering angle
    - Choosing a chassis that enabled an angle range of 65°–95°, center 80°
    - Programming our code to allow these angles through use of servo motor

Speed choice: MOTOR_DRIVE = 1600 µs
Through use of testing, we concluded that this is the fastest speed in which we could consistently obtain the same results, navigating safely through the challenge. 1500 is its stationary position.

# Gif
<img width="204" height="283" alt="azhn33" src="https://github.com/user-attachments/assets/6a6a92f3-39ec-4679-ba64-87e0c55c8f9f" />

# Wiring Diagram(file also included above)
<img width="440" height="311" alt="image" src="https://github.com/user-attachments/assets/e3eac9ea-6c17-4ec7-8378-bedb8b33fbd1" />

# Power and Sensor Architecture
The Battery powers the Raspberry Pi, which is the master controller, through the Raspberry Pi's power regulator. The Raspberry Pi receives information from the Pi Camera. The Raspberry Pi then sends basic commands to the Arduino Nano which sends the information through the ESC into the servo and DC motors, which controls the steering and speed of the robot. 

Battery -> Power Regulator -> Raspberry Pi -> Arduino Nano -> ESC -> Servo & DC Motors

In choosing our sensor, we decided upon the the Pi camera because we deemed it would work well with the Raspberry Pi and had all the necessary functions such as a wide camera view and colour differentiation enabling it to see both the walls and pillars

# Software Architecture & Obstacle Strategy
## Open Challenge
For the Open Challenge, we have 2 modes; TURNING MODE and FOLLOW_WALL MODE. 
### FOLLOW_WALL MODE
During the wall following mode, the robot uses a combination of PD control and the Pi cameras input of the amount of the walls within its view in order to calculate how close to each wall the robot is, adjusting the robot as it goes. It automatically knows that the the bigger wall is closer and should move towards the smaller of the 2. To achieve this, we had 2 ROI(regions of intrest) on each side of the camera in which the camera would detect the amount of black, Thus knowing the ratio of the distance to each. In using this ratio, we could calculate what needed to be done in order to get to the middle. Originally, we set the kp(present error) = 0.5 & kd(predicted future error) = 0.1 but after further testing, we concluded that the robot didn't adjust quickly enough so with testing, we concluded that the perfect numbers were kp = 0.7 & kd = 0.3. 
### TURNING MODE
On the other hand, TURNING MODE is used once one of the walls disappears completely. Once this is the case, we know that the robot has reached a point where there is no wall on one side and so the robot starts turning for a few seconds or until the wall is visible again on the side where it previously wasn't. This allows us to preform the sharper turns without having to increase the kp & kd values to ones that may have the robot overcorrect. 

## Obstacle Challenge
Meanwhile, for the Obstacle Challenge, the Pi camera notifies the pi of the closest pilar, whether red or green. When the pi receives this information, it shifts over to one side of the track by increasing the amount of wall the camera should be seeing on one side compared to the other. 

## Lap Counting
Meanwhile, the robot also counts the laps using the blue and orange lines. after having counted 23 blue and orange lines, the robot enters a time of "STOP_DELAY" in order to stop not at the line located in the corner of the map but at the intended side of the map. 
In the end, all of the motor commands are sent to the Arduino Nano as simple commands, only stating whether it is a command to the servo motor or the DC motor and what degree/speed it should be set to. 


# Systems Thinking & Engineering Decisions

## Arduino ↔ Raspberry Pi Split

We divided the robot's jobs between the Arduino and Raspberry Pi based on what each device is good at:
- Arduino: Handles the IMU and controls the motors using PWM.
- Raspberry Pi: Handles the camera and computer vision using OpenCV.

The Arduino is better for tasks that need very accurate timing. The Raspberry Pi runs Linux, which can interrupt programs, so it cannot guarantee the precise microsecond timing needed for reliable PWM control. On the other hand, the Arduino does not have enough processing power or the software environment needed to run OpenCV effectively.

This split lets each device focus on the job it is best suited for.

## IMU Drift Mitigation

Gyroscopes can slowly become inaccurate over time because of small measurement errors, known as gyro drift.

To reduce this, the robot calibrates the gyro at startup by taking 2,000 samples over about 6 seconds while the robot is not moving. This gives us an estimate of the gyro's normal error, which can then be removed from later measurements.

We also use a dead-zone where values with abs(z) < 0.1 are treated as zero. This stops very small amounts of sensor noise from being interpreted as real movement.

The tradeoff is that extremely small rotations might be ignored. However, this is acceptable because we care more about reliably detecting larger turns than detecting tiny movements.

## Color Space Choices

We use different color spaces for different vision tasks.

- LAB: Used for detecting walls. It handles changes in brightness better, making wall detection more reliable under different lighting.
- HSV: Used for detecting colored pillars and lines. It makes it easier to isolate specific colors based on their hue.

Using different color spaces is a tradeoff: instead of using one method for everything, we chose the method that works best for each type of object.

## Iteration and Improvements

The robot was improved through repeated testing and changes shown in the commit history.

Some important improvements included:
- kp and kd tuning: Adjusted the PID controller to make the robot steer more smoothly and accurately.
- Reduced ROI height: Focused vision processing on the most useful part of the image.
- Added IMU turn confirmation: Used the IMU to help confirm that a turn actually happened instead of relying only on the camera.
- Added pillar detection: Improved the robot's ability to recognize important objects in its environment.

These changes show an implement → test → improve process. Instead of designing everything perfectly from the start, we used testing and observed problems to guide improvements.
