# STM32-Quadcopter

Work in progress. It does not fly yet, needs more work done to make it keep upright in the air.

STM32F411, MPU6050, QMC5883l, BN357, BMP280, NRF24l01, Second STM32F411 SD Logger

Link to SD logger repo: https://github.com/Rokas-embedded/Logger

## How it looks like

### Controller:
![](./images/controller_with_sd.jpg)

### Frame top:
![](./images/frame_top.jpg)

### Frame bottom:
![](./images/frame_bottom.jpg)

## Video of test 3 with problems
YT link: https://www.youtube.com/shorts/ODtNJZJHQQQ
![Test 3 picture](./images/test3_image.png)


## TODO lists

TODO after test 1:
* X Take off testing equipment as drone is too heavy to fly.
* X Uncomment second axis stabilization

TODO after test 2:
* X fix stuttering issue with motors. I think it is from receiving too little radio signals
* X figure out how to make screw nuts not undo themselves. Make the whole thing more rigid.
* X maybe upgrade to the extended range radio sensor
* X Make the robot turn off if it gets to an angle of like 45 degrees as that is not helping it hover

TODO after test 3:
* X Fix issues with remote restarting and roll joystick not making contact with pins.
* X Fix quadcopter event loop lag. Improve refresh rate of robot
* ! figure out how to use integral without super windup
* X remove comments from both remote and quadcopter.
* X add loop timing to remote to have constant refresh rate
* ! Implement logging to sd card and log in a format that can be understood by something like betaflight blackbox analyzer