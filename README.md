# WRO-future-engineers
WRO 2026 Andrew and Nathan

#Materials
Originally we planned to use the following RC car for its chassis: https://www.amazon.ca/Full-Scale-Rear-Wheel-Rechargeable-Control-Crawler/dp/B0G24MWNGD?th=1
However, this did not have the things that we wanted so we switched chassis

In addition, we used this 2D LiDAR: https://www.aliexpress.com/item/1005003012681021.html?spm=a2g0o.order_list.order_list_main.31.67dd18029q37Y0

This Pi camera: https://www.aliexpress.com/item/1005009258177201.htmlpdp_npi=6%40dis%21USD%21%2430.99%21%2415.19%21%21%21%21%21%402101e83017699792506122817ed50e%2112000048516190389%21btfpre%21%21%21%211%210%21&afTraceInfo=1005009258177201__pc__c_ppc_item_bridge_pc_related_wf__q8i25HV__1769979250883 

This Pi Power Regulator: https://ca.robotshop.com/products/yahboom-power-supply-expansion-board-raspberry-pi-5 

This Battery/Power Supply: https://genstattu.com/gens-ace-1300mah-2s-7-4v-45c-g-tech-lipo-battery-pack-with-deans-plug/?srsltid=AfmBOoo-qPXzcxuH2dIqTfVYg5ghG9WdKi2b53X-R9M8j3XF_JQlLKJL 

This Arduino Nano: https://store-usa.arduino.cc/products/nano-33-ble-sense-rev2 

This Button: https://www.amazon.ca/dp/B01J9KO7DC?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1

#Mobility & Mechanical Design
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


