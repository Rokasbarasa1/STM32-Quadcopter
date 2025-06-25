# STM32-Quadcopter

![IMG_4697](https://github.com/user-attachments/assets/5bd907f9-0777-416a-9e56-f72a6bac76c3)

I am making a quadcopter from scratch (firmware including). This is the second step after my [Self balancing robot](https://github.com/Rokasbarasa1/ESP32-self-balancing-robot) project. It is still a work in progress, but a lot of the heavy lifting has already been done.

### Some features:
* Acro and Angle modes.
* GPS position hold flight mode (Work in progress).
* Altitude hold flight mode (Work in progress).
* Controled by my custom [Pi-pico based remote control](https://github.com/Rokasbarasa1/Pi-Pico-remote-control). 

### Sensors and hardware used for controller:
* Controller mcu - STM32F411
* Accelerometer + Gyro - MPU6050
* Magnetometer - MMC5603 (Used QMC5883 before)
* GPS - HGLRC M100 5883 (Used BN357 before)
* Barometer - MS5611 (Used BMP280 before)
* Radio link - RadioMaster RP1 2.4GHz ELRS (Work in progress)
* Debug Radio communication - NRF24l01
* Logging - [My custom STM32F411 based SD Logger](https://github.com/Rokasbarasa1/Logger). 

### Hardware of the drone itself:
* Motors - 4 x EMAX ECO II Series 2807 Motor
* ESC's - 4 x HGLRC 30amp 3-5S Dshot600 BLHeli_S 16.5 BB2 ESC (F-H-40 flashed with BlueJay 0.21.0V for BDshot600)
* Proppelers - 4 x HQProp DP 7x3.5x3 V1S 3 blade Propeller
* Battery - CNHL Black Series 3200mAh 14.8V 30C 4S (also used Turnigy 2200mAh 3S 25C)

# Latest flight video (January 2025)

[![Watch the video](https://img.youtube.com/vi/jI-Cl2vByNU/maxresdefault.jpg)](https://www.youtube.com/watch?v=jI-Cl2vByNU)

# Pictures (Outdated)
![Frame top](https://github.com/user-attachments/assets/da05143e-058d-49e6-974b-7a364a5608be)
![Frame bottom](https://github.com/user-attachments/assets/6a92aa1c-1b76-4436-8c6b-8c039739f69b)
![Controller](./images/controller_with_sd.jpg)

### Checklist for going testing
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

# Design documentation (work in progress)


## Controller 

The controller is the main piece of the system. It is much more limited in what it can do than a normal computer, it is smaller, slower and resource constrained. So it cannot run a full GUI operating system. The good thing about them is that they are cheap and, depending on which one you get, just powerful enough to do interesting things. They are also cheap, I got mine for 7.28 USD. The microcontroller itself is just a chip on the breadboard. It is very self sufficient, sometimes you can just supply them stable power and they work fine. Like the attiny85 that just plugs into a breadboard. 

We need one that is more powerful than that so we will go with one called the STM32F411. ST(M)icroelectronics is the manufacturer, the 32 bit is how big data chunks the system can handle, and the F stands mainstream performance line of their chips, 411 is just a serial number for features. We dont really care about the features that much, it is maninstream so it will work for most things. 

I went with a ST microcontroller as it was one I had not used before. The chip on this one is very small so it is mounted on a pcb to seperate the tiny pins on the chip into big pins that we can use. The PCB that it is mounted on gives the microcontroller name blackpill because its a black pcb and its small. The pcb also has some extra features like voltage regulators to protect the chip, external oscillators to speed it up and leds to help us indicate status later. The pcb is so small that it lacks some quality of life features that a big pcb would have, like a programmer. The chip has to instead be programmed usng another product of ST called ST-link V2, which just converts a Usb interface to a programming interface for the stm32, it uses 4 pins to do that. This is negative of ST's products, other manufacturers dont have this problem with small pcb's, most of the time. The upside is you can debug the code with the same product, this is something that is painful on other microcontroller brands. The small form factor of the pcb is also very helpful for integrating the microcontroller into projects.

Each microcontroller manufacturer has their own "library" that abstracts the difficullt hardware drivers (HAL), on top of which the coding of the microcontroller can be done. In the case of the STM32 it is STM32Cube which lets you use C or C++ to write code for it. You can use Rust or anything else to write the code but then you are not using the manufacturers libraries and have to relly on open source. In my case I went with C as it is very simple and I can get great performance out of it. 

_The STM32F411 has a lot more GPIO than ESP32, which maybe has half of its pins unavailable and a bit more than the Pi-Pico on which a lot of the pins are grounds. Its also easier to mount on a breadboard than the ESP32._

_I hate the CubeMX code generation, feels like there is just junk in my repo. The alternative is really badly documented, to the point of needing to go into the library code to figure out how to use it. Feels like they skimped out on actual documentation to make you use the shitty code gen. Using the library code to figure something out is not easy either as there are multiple functions with similar names or functions that are depreceated. It always confusing to set anything up from scratch, like spi or i2c, there are so many functions that are supposed to do the exact same thing. By far the worst experiance of developing embededd software. Even fucking atmelavr felt like butter compared to this, though that might be a high bar in the embedded world. You cannot have development velocity in this ecosystem if you touch even a bit of the ST library. ESP32, Pi-pico and atmega are a lot better at this._

_Flashing the firmware on it is a lot easier though, in cases. If i was working on a breadboard, the Pi-pico would be a lot easier. When i made a protective cover for the controller part of the drone it was easy to press the pins of the programmer agains unsoldered holes of the pcb for programming. I did not have to fish for the 4 pins to attach too. This meant that sometimes the programming would not work, but overall it was a good experiance._

_For performance the STM32 is deffinetely not as fast as the ESP32, but it is fast enough, most of the performance is lost on sensor communication anyway.*

### Code design decisions

_The code looks not as a usual developers code looks like. I like having full names of the variables, no such bullshit as dT for delta time or whatever one letter variable. The same is for functions, full verbose names. I like to come back to old features and understand what is happenning. Lots of comments also._ 

_If there was a prettier implementation for C (C code formaters are garbage, all of them) I would use it to enforce the same syntax everywhere, but there is not so there are gaps. I would love to enforce breaking of function variables if the developer has broken at least one, curly braces in line with for loops and if statments and functions.

_I tried to seperate the code into modules, of radio, data getting, motor control/pid and logging. They need to be moved to different files like controller/services/models architecture on backends, but at the moment it is just so much easier to have everything logic related in one file. In the future will be, main.c/business logic/drivers, with main.c only being able to reach drivers if it goes trough business logic. I come from full-stack development and we have much better understanding of code organization than embedded developers. The code will be seperated into seperate files once I am done with the project. I dont care about clean code, the functions will not be made 5 lines long._

## Sensors drivers
To make our drone sense the world around it we need sensors. Based on the output of these sensors the code that i wrote in the microcontroller will make decisions and perform further actions to manipulate the environment it is in.


_A note about the sensor driver implementation. I continued the use of enums for defining the registers of the sensor and so on. Makes reading of the sensor code and even configuration of it a lot easier._

_Usage of macros for constant definitions is also done now. No more magic numbers, they have names now. Some pre division of these values is also done._

### I2C and sensors in general
The sensors are just little microcontrollers in most cases, they have logic and interfaces that let the data be extracted from them digitally(toggling a single pin or more from 3.3v to 0V). There are sensors that are analog, but we are not using them, they are harder to work with. For this communication we will use I2C. I2c is just a recipe (protocol) for what way to toggle the pins and in what order that both the controller trying to get the data (master) and the sensor (slave) understand and adhere to. There are other protoocls as well that the sensors support, like SPI, but I chose this one because it is easier to deal with.

To do things with a sensor you need to read its datasheet. The most important part of which is the register map, it tells you where everything is stored and what register addresses you need to give to the sensor to read out the data from the registers. With i2c this often looks like sending the address of the register then reading the number of bytes you expect the data to be. Sometimes these sensors dont just automatically fill up their registers with data and you need to actually send commands that trigger actions that fill up populate those registers. Sometimes these commands just activate some special ability of the sensor. Thes sensors are almost always configurable in the same way also, by sending what register you want to fill up and then sending the bytes of data you want to put in that register. Configuring the sensor can be though of as having a wall of switches (the register map). There are as many rows of switches on it and the rows correspond to registers, each row has a number of switches that correspond to how large the register is in bits (most often its 8 bits). Each switch in the row does something, each one has a long description of its functionality in the datasheet. Some of the rows of switches you cannot manipulate as they are the ones storing the measurement that the sensor does, so the analogy falls appart there.

It should be noted that sensor data should never be taken as reliable data, the sensor data by default is always miscalibrated and has noise in it that makes judgements based on the data difficult. Each sensor type also has some problem that makes complex systems development difficult, but there are solutions to this, that we will talk about in sections "sensor fusion" and "filtering".

_I noticed that the STM32 did badly with an interrupt happenning while I2C communication was happenning. The i2c communication would freeze and take up 100 ms each one. For that reason all interrupts are disabled before any i2c communication. I put that in main.c but I should probably do it in each sensor driver._

_I should have not used the I2c bus anymore in this project, for any of the sensors, its a bit too slow. I went above the recomended bus frequency though and reached 1.6 MHz but went down to 1.0 Mhz for stability just in case. That saved quite a of cpu time._

# Gyro
The gyro can tell how fast the drone is rotating in each axis. If you tilt the drone 90 degrees to be on its side after it was standing flat on the ground and you do it immediately in one second the gyro will tell you that it degrees per second rotation is 90 on that axis you were rotating it around. To understand this, imagine a swing set attached to a metal pipe, the way the pipe is laying is some axis, lets say x, so the pipe is in line with the x axis. When you swing in the swing set you are really just rotating around the x axis of the pipe.

The gyro is the most important sensor for a drone. Depending on your requirements for the drone, it may be the only sensor you need, specifally for a mode control for the drone called acro mode. We will be implementing this mode of control but also more.
DRIFT

I am using the gyro on a sensor module called MPU6050. It is a common sensor, used on many drones. It actually has an accelerometer on it as well, but that is for later. The MPU6050 chip itself is mounted on a pcb that has voltage control stuff and power stabilization capacitors on it. This makes working with it easier, low risk of frying due to high voltage and it should be isolated from the power disturbances that can happen in an embedded system (these can just make the sensor not work very good). 

The sensor itself is configured by the code to be able to sense a max of 500 dps/s of rotation, which should be plenty for our drone if we dont plan to do spinny tricks in acro mode.

The gyro does have a problem. You may think if you keep summing the dps data over time, that you will get some data that tells you the orientation of the drone in 3D space, but that is not going to happen. The Gyro data is inherently noisy and if you sum it you will get values that drift over time. You can ease this noise issue by calibrating the gyro. Which is just reading maybe 1000 gyro values while sitting still and, finding the average gyro output value and then every time the you get the value out of the sensor you substract that average value from the data. This effectively removes some of the noise, but certainly not all of it. The next sensor will help deal with the noise issue.

### Accelerometer
The accelerometer can tell which way down is. You can imagine it like a a plank of wood in the air with a string tied to it and at the bottom of the string is a weight. As you move the the rotate the plank the string just keeps pointing to the ground. That is one property of the accelerometer, it can tell which way the ground is by displaying the force of gravity in G's split over 3 axies, x, y and z. So if the plank is at an angle to the ground, the G force will be split among the axies. If the plank is flat to the ground, only the z axis will have the value. If you make suddent movements and move some distance with the plank the weight will react to that and start to point away from down and will instead point slightly to the side. Using this you can sense when acceleration happens. This is where the analogy falls appart as the weigt will then have momentum, where as the real accelerometer will not be succeptible to that. When the acceleration stops the accelerometer will also stop being affected by it. ![board with weight](https://github.com/user-attachments/assets/ff592a4f-7404-480f-ac9d-0533faf04fab)

So now we know which way is down and we know if we are accelerating in some direction.

The accelerometer also has some problems. If we want to use the data in any way on something that moves and can control itself we will always have parasitic accelerations as its just picks up every movement and creates more acceleration data. The accelerometer also needs calibration, its calibration is in part similar to the gyro, but can go further. The same average value calibration can be done when the drone is flat (extremely flat, like bubble level flat in both axies), this gives you the deviation form actual gravity. If any other axis than z has average values of more than 0.01 then that needs to be substracted from them. If the z axis is not exactly 1.0 G then it needs to also be substracted or added to to make it 1.0. This is the first level of calibration. 

The second level is a bit more complicated, it can happen that when you rotate and try to isolate the axies (make the value only show on one axis) that value can be not exactly 1.0. If you were to rotate the drone around for 5 minutes straight without introducing ANY acceleration but gravity and logged all of that data and plotted it in a python 3 dimensional graph, you would see either a sphere or a ellipsoid(squished sphere). The shpere means youre accelerometer is probably good to go, but even a slight ellipsoid squish means its bad. To fix this type of problem we need to use linear algebra and use matrix multiplications to transform the data into the sphere. For this I used a software called magneto12, I give it the raw values of the accelerometer and it calculates me how the modifications to the values. These calibrations are then applied every single tiem the acceleration data is gotten.

![Sphere](https://github.com/user-attachments/assets/04a7487c-2fcd-454e-af4f-a5ff75fd2d5f)
![ellipsoid](https://github.com/user-attachments/assets/78caa9cb-d260-42d0-add7-f300ce37a8aa)


The accelerometer combined with the gyro though solves this issue. We will talk more about this later, but gyro+accelerometer can give us accurate data on our orientation for axies x and y, but not z. This data is enough to make a drone that has a flight mode called angle mode. The next sensor will solve that problem of the z axis.

The accelerometer we will be using is on the same chip as the gyro, the MPU6050. We will use the 2G range of readings for the accelerometer. Having a lower g value capture means there is more bits of data left for higher resolution value storage (higher resolution means you just have more digits after the decimal).

### Magnetometer
The magnetometer can sense something about the magnetic field of the earth and it gives the readings in micro teslas in 3 axies. That is as much as I know about how it works. It is succeptible to AC electicity wires and metal objects, that is why its best utilized outiside in nature. It is even affected by the matal that is on the drone. That is why the calibration on this is essential. The calibration of it is largely the same as the accelerometer, except that there is only the second more complicated calibration that needs to be done. It is a lot easier than the accelerometer to perform the calibration as it is not succeptible to movement. The sphere and the ellipsoid plots are also the same but there is one extra plot type that lets you know if your sensor is usable or not. In my case a a lot of my magnetometer modules were broken and the resulting plots looked more like pancakes rather than spheres or ellipsoids. This means the sensor is toast and there is not much use in it. I had this happen to me when i was using an older sensor called QMC5883L which i got from AliExpress (the only broken thing i got from AliExpress). I replaced it with a different model and it was working fine. The resulting calibration values that you get from the magneto12 software are actually 2 different types. The first one is 3 values which is for the hard-iron distortion in the data. This type of distortion is caused by magnetic materials, motor magnets for example, and is the easiest to fix. The second source of distortion is called "soft-iron", this is caused by materials that distort the magnetic field of the earth, like iron, nickel and so on. The calibration values for the "soft-iron" distortion is the same one that we used for the accelerometer, which is 3x3 grid of values in an identity matrix (linear algebra) the values of which are a bit corrected to correct our data when the calibration is applied. Calibrating the magnetometer on the drone also requires the drone to be fully assembled, that way all the distortions in the materials used in the drone are present and can be calibrated away.

The magnetometer does not have any other weakness besides being affected by its surroundings. Its readings can be used to find the north direction (will talk about this later), which lets the drone sense its z axis orientation in 3d space. The magnetometer is very useful if you want to keep a constant heading when moving, but is also useful for GPS position hold functionality of the drone.

### GPS
The GPS is technically not a sensor but a receiver antena for sattalite signals, but i like to treat it like a sensor.




### Axies
Explain X Y and Z. 

thilike a string that has something heavy tied to it, as you move 

The accelerometer can be used to solve the issue with the gyro 
To get more advanced functionality out of the drone, more sensors are needed. To get what is called angle mode
This is the first sensor 

To make the drone know which way is down 
I used the MPU6050 again, it works fine. I did add more calibration to it. I learned form the magnetometer driver development that the way the sensor feels the gravity can be distorted, it can be not a perfect circle, but something oval. For that you need to get a lot of measurements and do the same "hard-iron" and "soft-iron" calibration as for the magnetometer. I have to admit that i did not go all out with the "soft-iron" equivalent of calibrating the accelerometer, I did a half-assed job and I kept that. I did make sure to do the "hard-iron" style calibration good though.  

### Gyro
Nothing changed. Still using MPU6050 gyro. It works good. I did not even consider using a different gyro. It is much more important to filter the data. 

### Magnetometer

The QMC5883L caused me so many issues. They are the worst module i have received from AliExpress, consistently. They all initialize  and give values but they are broken. One axis always does not work and it is easy to see that in calibration of it, a straight line of values or a cloud with no hard edge. There was nothing that could be done to fix this, not even taping the sensor with a strong magnet, that would shuffle the brokenness a bit but it was always one axis stuck anyway. I ended up switching to the MMC5603

## Other hardware

BREADBOARD



### Old todo lists i will remove later

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
* X Add accelerometer calibration tweaking to the remote control so the drone does not have to be reflashed every time
* ! Consider switching to the kalman filter for better sensor data


Test 10
The new feature of the remote control being able to change the accelerometer offsets was really good. I managed to get a general direction in which it needed to be adjusted, it always leaned right and a bit forward. I was not able to adjust it perfectly though as i ran into issues of my pid settings just not being good enough, P and D are no longer enough to keep good balance. The new flight mode did not work as expected, i set it lower than what is generally expected from it and i did see that it was not enough, i needed to make a big demand of 70 cm for it to star rising a bit, which was not good. Need to have more confidence in it to not go flying somewhere, but possibly try it more after i figure out balance on the mode 0 balance. Low pass filtering of the gyro and accelerometer did not really have a negative impact on the quadcopter. Tried playing around more with the integral with a bit of success in lowering the oscillation of the drone when balancing. Need to continue tests on a very flat surface next as moderately level ground is not enough anymore. The beta in the 

TODO after test 10
* Do more tests in flight mode 0 with integral and when a better balance PID is found adjust accelerometer offsets again.
* Add a note in the remote control accelerometer calibration screens what the impact of the calibration will be, like - +X = Pitch forward and so on.
* Compare to other quadcopter projects to see what i am missing to make things better.

Test 11
Realized that offsetting the actual accelerometer calibrations might not be the best way to fix the quadcopter balance point. Tried implementing a degrees offset for the pitch and roll of accelerometer but ran into issues of feedback loops once the offset is applied, making one or more motors spin faster and faster when idle on the ground until they shut off. Meaning that the gyro_degrees is affected in a feedback loop. (The feedback loop turned out to be related to the mpu6050 outputs of acceleration and gyro freezing)

TODO after test 11
* Figure out problem with feedback loop of offset degrees
* Replace MPU6050 with new sensor as this one is getting frozen sometimes. It is fine to use for testing now, but for extended flights in the future it will fall from the sky.


Test 12
I did some more tests and i found out that there is a problem with using calibration and filtering. If calibration was done on data that was not filtered and then the actual data used is the filtered one, the effect is that the calibration is useless. I have to either apply filtering on the sensor hardware level or use filtering everywhere in software including when calibration is done. Durring testing i saw that the gyro values post calibration were a constant 3 degrees/s which made half of the flight test useless as the battery was half drained. 

I also found that that the motor and prop vibrations really do affect the sensor readings a lot, i am surprised it even hovers a bit after looking at them at 40% throttle.

TODO after test 12
* ! Add material dampening of vibration
* ! Improve software filtering to remove what vibration was not already removed by material dampening.
* ! Test vibrations with fresh propellers.

Test 13
Tested the new low pass filtering on the drone. It worked very good at the 45Hz frequency setting, not so good at 22Hz or 10Hz with significant delay in response being visible. Did more tests on the yaw control functionality, it works very well now but there is a big drop in throttle when yawing with at a fast rate. The altitude hold functionality in flight mode 1 works better now but is still no good as i still need to put the throttle stick to 75% for it to do anything and lift off it does not keep the altitude at a neutral position. The balance correction is now easier with the low-pass filter but it still needs some work to find the sweet spot.

TODO after test 13
* ! Find out if the yaw control throttle issue with altitude could be solved or if that is just because my quadcopter does not have enough power in to keep altitude with two motors.
* ! Improve the offsets more on the next test.
* ! Find out why the altitude is not working so good.

Test 14
Got a better balance point but when testing the altitude hold broke one of the legs.

Test 15
Dialed in the altitude hold settings for PID, the quadcopter now holds altitude relatively well, but as it struggles to find a level offset it is not good to improve it more at the moment. The offsets still change a lot even with a low pass filter on, need to invest more in flight-controller dampening.

Test 16
Mounted new motors and tested them. Much less vibration as can be seen in the vibration profile for them. The quadcopter flies a bit better, but not as much change as i though it would have. I tried my best to balance it so it does not drift and got closer than the other motors, but did not get results that are much better. I am no longer even sure that this is a goal worth doing more tests for, and maybe i should just get the best stabilization i can and move on to perfecting the altitude hold and gps hold.

I tested it again the same day. Managed to improve the altitude hold functionality, by raising the integral gain 12 times I sped up the time it takes for it to charge up, so it reacts faster from ground launch. An issue is that raising the integral gain to charge it faster makes the integral give some overshoot to the vertical velocity target, for this i raised the proportional by 0.2 and got it to control a bit. Further testing is needed.

I did notice that with the new prop and motor combo the quadcopter does not have as much battery, so it means it is less efficient. This is expected as the propellers are smaller. I have also found that the new propellers are more durable than the ones i had before, maybe the decreased tip speed, material and shape of the tips plays a role in that. The quadcopter feels more loose with the new propellers, and that is not necessarily a good thing.

TODO after test 16
* ! Tighten all the nuts on the legs
* ! Find out why when gps is enabled the flight controller starts having a 140ms delay in sensor readings in I2C after the thing being on for a couple of minutes. 


Test 17

I have hit a dead end in testing. The drone is not supposed to handle this bad, vibration has been eliminated. I believe i am missing something big in regards to PID. I have seen others implement acro mode first and then feed that to implement angle mode. I am going to try to do the same as i have planned to implement acro mode anyway.

Test 18

I was correct in thinking something was missing. As soon as i started testing with pid as angle mode -> acro mode -> motors things started to work great. The drone now balances a lot better in angle mode and is very easy to control compared to before. Acro mode works ok, i can see some wobble as it wants to settle down to to a angular rate, which means there is more tunning to do. The next step will be to get the sd card blackbox logger working again, by fixing the hardware problem it has. Using the logger I will be able to see how it responds to setpoints over many different pid values for the acro mode and in that way i can figure out which one fits it best.

TODO after test 18
* X Fix the logger hardware problem.
* X Add more details to the betaflight log about the settings of the quadcopter, so that it allows the data to be analyzed more.
* X Make it so that when PID settings are changes, the quadcopter starts writing a new blackbox log file.
* X If the quadcopter starts a new log file it has to add the ending to the old one, so that opening it is easier on the computer.
* X Tune the blackbox log to the point that the log corresponds correctly to drone orientations
* ! Perform tests as in Chris Rossers's video on PID tunning.
* X Fix broken legs.
* ! Try flashing BlueJay to the ESC's to have rpm data from the esc's



Fix roll and pitch joystick
Fix scaling of ALL degree values
Fix right side motors not showing.
The fft graphs don't show throttle vs frequency noise plot. No Throttle detected




