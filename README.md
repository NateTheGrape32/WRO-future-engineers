# WRO-future-engineers
WRO 2026 Andrew and Nathan

# Materials
Originally we planned to use the following RC car for its chassis: https://www.amazon.ca/Full-Scale-Rear-Wheel-Rechargeable-Control-Crawler/dp/B0G24MWNGD?th=1

However, this did not have the things that we wanted so we switched chassis to this one: https://www.aliexpress.com/item/1005005831438904.html?src=google&pdp_npi=4%40dis%21CAD%21137.79%2167.99%21%21%21%21%21%40%2112000034508242366%21ppc%21%21%21&snpsid=1&src=google&albch=shopping&acnt=272-267-0231&isdl=y&slnk=&plac=&mtctp=&albbt=Google_7_shopping&aff_platform=google&aff_short_key=UneMJZVf&gclsrc=aw.ds&albagn=888888&ds_e_adid=&ds_e_matchtype=&ds_e_device=m&ds_e_network=x&ds_e_product_group_id=&ds_e_product_id=en1005005831438904&ds_e_product_merchant_id=5551326180&ds_e_product_country=CA&ds_e_product_language=en&ds_e_product_channel=online&ds_e_product_store_id=&ds_url_v=2&albcp=20695954287&albag=&isSmbAutoCall=false&needSmbHouyi=false&gad_source=1&gad_campaignid=20705699266&gbraid=0AAAAAoukdWPTzEickRzoimrnL1MwwtHVW&gclid=CjwKCAjwspPOBhB9EiwATFbi5NuBWxbRLo5owD-vEzh-nllFJE7xHkzB1mxMaOIA7-0v5My0EEMAEhoCYgAQAvD_BwE 

In addition, we used this 2D LiDAR: https://www.aliexpress.com/item/1005003012681021.html?spm=a2g0o.order_list.order_list_main.31.67dd18029q37Y0

This Pi camera: https://www.aliexpress.com/item/1005009258177201.htmlpdp_npi=6%40dis%21USD%21%2430.99%21%2415.19%21%21%21%21%21%402101e83017699792506122817ed50e%2112000048516190389%21btfpre%21%21%21%211%210%21&afTraceInfo=1005009258177201__pc__c_ppc_item_bridge_pc_related_wf__q8i25HV__1769979250883 

This Pi Power Regulator: https://ca.robotshop.com/products/yahboom-power-supply-expansion-board-raspberry-pi-5 

This Battery/Power Supply: https://genstattu.com/gens-ace-1300mah-2s-7-4v-45c-g-tech-lipo-battery-pack-with-deans-plug/?srsltid=AfmBOoo-qPXzcxuH2dIqTfVYg5ghG9WdKi2b53X-R9M8j3XF_JQlLKJL 

This Arduino Nano: https://store-usa.arduino.cc/products/nano-33-ble-sense-rev2 

This Button: https://www.amazon.ca/dp/B01J9KO7DC?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1

Servo motor for driving: https://hitecrcd.com/hs-5055mg-economy-metal-gear-feather-servo/ 



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
The Battery powers the Raspberry Pi which is the master controller, receiving information from the Pi Camera. The Raspberry Pi then sends basic commands to the Arduino Nano which sends the information through the ESC into the servo and DC motors, which controls the steering and speed of the robot. 

In choosing our sensor, we decided upon the the Pi camera because we deemed it would work well with the Raspberry Pi and had all the necessary functions such as a wide camera view and colour differentiation enabling it to see both the walls and pillars

# Software Architecture & Obstacle Strategy
For the Open Challenge, we have 2 modes; TURNING MODE and FOLLOW_WALL MODE. During the wall following mode, the robot uses a combination of PD control and the Pi cameras input of the amount of the walls within its view in order to calculate how close to each wall the robot is, adjusting the robot as it goes. Originally, we set the kp = 0.5 & kd = 0.1 but after further testing, we concluded that the robot didn't adjust quickly enough so with testing, we concluded that the perfect numbers were kp = 0.7 & kd = 0.3. 

Meanwhile, for the Obstacle Challenge, the Pi camera notifies the pi of the closest pilar, whether red or green. When the pi receives this information, it shifts over to one side of the track by increasing the amount of wall the camera should be seeing on one side compared to the other. 

Meanwhile, the robot also counts the laps using the blue and orange lines. after having counted 23 blue and orange lines, the robot enters a time of "STOP_DELAY" in order to stop not at the line located in the corner of the map but at the intended side of the map. 
In the end, all of the motor commands are sent to the Arduino Nano as simple commands, only stating whether it is a command to the servo motor or the DC motor and what degree/speed it should be set to. 
