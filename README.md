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


ESC name "Cheetah HW30A Brushless ESC"
BEC from ESC powers the flight controller

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
* X Implement logging to sd card and log in a format that can be understood by something like betaflight blackbox analyzer
* X Make sure the motors never stop spinning at even at the lowest setting when controlled by the remote. This will help responsiveness.

TODO after test 4:
* ! Add blackbox logging for gps data
* X Find out how to adjust what pwm frequency the esc accepts and change it from 50Hz to as fast as possible.
* X The radio module caches the data sent durring the boot proccess of the quadcopter. Make sure the data is deleted before it gets to the pid loop as it is dangerous.
* X Recalibrate accelerometer because it was moved and reglued.

TODO after test 5:
* X Tie down new escs like the old ones were
* X Make sure the motor screws don't undo from vibrations
* X Mark the front and back of the drone
* ! Fix issues with magnetometer
* X Fix blackbox log roll log being opposite
* X Reduce angle of attack for pitch and roll

TODO after test 7:
* ! Fix issues with blackbox logging showing wrong motors spinning and wrong pitch and rol. All of them either inverted or showing up on the wrong motor.
* ! Find a new more reliable magnetometer
* X Attach an FPV camera for better orientation and control while flying.

TODO after test 8 (Crashed and broke a rotor leg):
* X Print new rotor leg parts
* X Resolder cut ESC wires
* X Flip degrees per second yaw control application to motors and reduce the gain.

TODO after test 9:
* ! Continue doing small tests to see which way the robot is leaning with no pitch or roll inputs to calibrate the accelerometer

Test checklist:
* Drone frame
* Controller
* Controller antenna
* Propellers
* Sd card adapter
* Micro sd card
* Pliers 
* Wire cutters
* Screwdriver 
* Screwdriver bits
* Remote control
* Laptop
* Usb extension cable
* STM32 programmer
* Charged battery
* Battery checker