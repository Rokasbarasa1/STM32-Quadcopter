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

<!-- 
_The STM32F411 has a lot more GPIO than ESP32, which maybe has half of its pins unavailable and a bit more than the Pi-Pico on which a lot of the pins are grounds. Its also easier to mount on a breadboard than the ESP32._

_I hate the CubeMX code generation, feels like there is just junk in my repo. The alternative is really badly documented, to the point of needing to go into the library code to figure out how to use it. Feels like they skimped out on actual documentation to make you use the shitty code gen. Using the library code to figure something out is not easy either as there are multiple functions with similar names or functions that are depreceated. It always confusing to set anything up from scratch, like spi or i2c, there are so many functions that are supposed to do the exact same thing. By far the worst experiance of developing embededd software. Even fucking atmelavr felt like butter compared to this, though that might be a high bar in the embedded world. You cannot have development velocity in this ecosystem if you touch even a bit of the ST library. ESP32, Pi-pico and atmega are a lot better at this._

_Flashing the firmware on it is a lot easier though, in cases. If i was working on a breadboard, the Pi-pico would be a lot easier. When i made a protective cover for the controller part of the drone it was easy to press the pins of the programmer agains unsoldered holes of the pcb for programming. I did not have to fish for the 4 pins to attach too. This meant that sometimes the programming would not work, but overall it was a good experiance._

_For performance the STM32 is deffinetely not as fast as the ESP32, but it is fast enough, most of the performance is lost on sensor communication anyway._
-->

<!--
### Code design decisions

_The code looks not as a usual developers code looks like. I like having full names of the variables, no such bullshit as dT for delta time or whatever one letter variable. The same is for functions, full verbose names. I like to come back to old features and understand what is happenning. Lots of comments also._ 

_If there was a "Prettier" implementation for C (C code formaters are garbage, all of them) I would use it to enforce the same syntax everywhere, but there is not so there are gaps. I would love to enforce breaking of function variables if the developer has broken at least one, curly braces in line with for loops and if statments and functions.

_I tried to seperate the code into modules, of radio, data getting, motor control/pid and logging. They need to be moved to different files like controller/services/models architecture on backends, but at the moment it is just so much easier to have everything logic related in one file. In the future will be, main.c/business logic/drivers, with main.c only being able to reach drivers if it goes trough business logic. I come from full-stack development and we have much better understanding of code organization than embedded developers. The code will be seperated into seperate files once I am done with the project. I dont care about clean code, the functions will not be made 5 lines long._
-->

## Sensors drivers
To make our drone sense the world around it we need sensors. Based on the output of these sensors the code that i wrote in the microcontroller will make decisions and perform further actions to manipulate the environment it is in.

<!--
_A note about the sensor driver implementation. I continued the use of enums for defining the registers of the sensor and so on. Makes reading of the sensor code and even configuration of it a lot easier._

_Usage of macros for constant definitions is also done now. No more magic numbers, they have names now. Some pre division of these values is also done._
-->
### Supplying power to sensors
These sensors that we will talk about and hardware are meant to be powered some way, unless they have a battery on board them they have to have a physical connection to a power source. This is usually 2 wires, one for ground and one for most commonly 3.3V or 5V. A lot of the older snesors use 5V, these are usually the sensors made for the microcontroller called Arduino. These sensors need 5V because the old Arduino uses 5V logic(5V for a set bit and a 0V for a not set bit). If communication needs to be done to the sensor then there will always be a minimum of 2 wires or there can be more to accomodate different methods of sending or receiving information relating to the sensors. In the case where something has its own power source, and you know the power source or at least the interface that you will be using utilizes the voltage that is the same as you are using, then connecting the ground wire is enough. In fact if the device you are connecting to uses a different voltage than yours it is also safe to connect your ground to its ground. The important thing to match is the voltage lines, 5V, 12V, 15V.  


### Communcication to sensors
The sensors are just little microcontrollers in most cases, they have logic and interfaces that let the data be extracted from them digitally(toggling a single pin or more from 3.3v to 0V). There are sensors that are analog, but we are not using them, they are harder to work with. There are many methods of extracting the data out of sensors, these are called protocols. Each protocol has its advantages and disadvantages. Different combinations of, speed, wire count and complexity in setting it up. I will not be bothered to get into the details of how each protocol works as there are actual books for that. The best way to see how it works though is by getting thing called a "logic analyzer" and seeing for yourself how your hardware works.

#### I2C
I2C is one of the most convenient protocols to settup, it uses two wires, one for data and one for clock signal, it can also be chained to multiple sensors/device. Its fast enough and you can go above the recomended speeds if you believe in your hardware setup. I used I2C on my self-balacning robot project for sensors and it was fine. I did that again on this project and that was a mistake. You have to understand unless you are setting up something fancy to not have to do the communication to sensors synchronously, I2C can waste a lot of time by not being the fastest method.

I2c is very easy to setup though, and rarely ever causes problems. On the STM32 i did find that interrupts (will talk about these later) have to be disabled for the communication to not break. When it did break it would take 100ms to error out and continue with the program, essentially breaking the whole flow of the time sensitive program.

#### SPI
SPI is a lot harder to setup than I2c. The devices that can be communicated to using this have to have a slave select pin, that when 0V is applied to it means that the device is being communicated with. It also has 3 other pins, one for clock, one for data coming to slave and one for data coming to master. When you try to setup the hardware for a SPI connection, there is always something that just does not work. Its like having to plug in a usb, you always do it wrong way first. Sometimes the slaves/sensors have an extra pin that is also names similar to the slave select pin causing additonal confusion. The benefit of SPI is the speed it can reach, if you really need to be at the peak of performance then this is what you need. There are even variations of the protocol that use double or quadruple the pins amounts for the slave-master pins to multiply the transmission speed.

SPI is also a very simple protocol that you can even implement yourself. That cannot be said about I2C.

#### UART 
UART is the last protocol we actually care about and is one that is not normally used for sensors, but for transmitting text information or large amounts of data. It is also a very simple protocol, it uses 2 pins and does not use a clock pin. It  also cannot be chained like I2C and it is most importantly asynchronous by default. It is considered slower than the other protocols but it has benefits. It is reliable over long distances of wire and has recover mechanisms 

### I2C and sensors in general
For this communication we will use I2C. I2c is just a recipe (protocol) for what way to toggle the pins and in what order that both the controller trying to get the data (master) and the sensor (slave) understand and adhere to. There are other protoocls as well that the sensors support, like SPI, but I chose this one because it is easier to deal with. I2c uses only 4 wires, 2 for data. I2c is usually used for sensors or things that dont nescessarily need max speed in communication. You sacrifice speed for ease of communication.


### UART
Uart is another protocol for communication. It again uses 4 wires 

To do things with a sensor you need to read its datasheet. The most important part of which is the register map, it tells you where everything is stored and what register addresses you need to give to the sensor to read out the data from the registers. With i2c this often looks like sending the address of the register then reading the number of bytes you expect the data to be. Sometimes these sensors dont just automatically fill up their registers with data and you need to actually send commands that trigger actions that fill up populate those registers. Sometimes these commands just activate some special ability of the sensor. Thes sensors are almost always configurable in the same way also, by sending what register you want to fill up and then sending the bytes of data you want to put in that register. Configuring the sensor can be though of as having a wall of switches (the register map). There are as many rows of switches on it and the rows correspond to registers, each row has a number of switches that correspond to how large the register is in bits (most often its 8 bits). Each switch in the row does something, each one has a long description of its functionality in the datasheet. Some of the rows of switches you cannot manipulate as they are the ones storing the measurement that the sensor does, so the analogy falls appart there.

It should be noted that sensor data should never be taken as reliable data, the sensor data by default is always miscalibrated and has noise in it that makes judgements based on the data difficult. Each sensor type also has some problem that makes complex systems development difficult, but there are solutions to this, that we will talk about in sections "sensor fusion" and "filtering".


<!--
_I should have not used the I2c bus anymore in this project, for any of the sensors, its a bit too slow. I went above the recomended bus frequency though and reached 1.6 MHz but went down to 1.0 Mhz for stability just in case. That saved quite a of cpu time._
-->
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
The magnetometer can sense the magnetic field of the earth, it gives these readings in micro teslas in 3 axies. That is as much as I know about how it works. The magnetometer can be used to always know in which direction is north, acting like a compass. You can use the relation to north to figure out which way you are facing or at least have a reference to where you were facing and which way you are facing now. The magnetometer is also essential if you want to use a GPS.

The magnetometer has some problems though, its data can be disturbed by magnetic fields of magnets and various metals. To use it, you need to always calibrate it, otherwise the data is useless. The calibration method is similar to the accelerometer calibration part 2. You collect a lot of data by rotating the drone in the air repeatedly, all kinds of loops, then you plug that inot magneto12 and you get two calibrations. A "hard-iron" calibration with 3 values and a "soft-iron" calibraiton with 9 values, really just a modified identity matrix. The "hard-iron" distortion is caused by magnets and the  calibration is just a offset of the value to center it around 0, similar to the gyro calibration. The "soft-iron" distortion is caused by various metals being around the sensor, it can also be characterized by the data not looking like a sphere but like a ellipsoid, just like the accelerometer one. It can also happen that you see a pancake in the "soft-iron" calibration, that means your sensor is not good and should be replaced, more on that later. All of these calibrations ideally have to be done outside in nature, far away from any disturbances. It is also very important that the drone is fully assembled when calibratin as each drone component can have some disturbance to the magnetometer.

I originally went with the QMC5883L magnetometer, it had a lot of problems with pancake graphs, but the ocasional one worked ok. When i started making my quadcopter I ripped the only working one that i had out of my self-balancing drone and it also turned into pancake. Eventually i had to get rid of all of them and just get a new sensor. I got the AdaFruit MMC5603 to replace it, origianlly I was really afraid that something was going to be wrong with it as the sensor chip was so tiny, but it was wrong and the sensor has so many more features. It is able to test the values it puts out itself and determine if something is wrong, set a data fetching rate, it can degauss itself which is amazing. Its such a great little sensor, it solves all of the problems i had with the magnetometers.


### UART

### GPS
The GPS is technically not a sensor but a receiver antena for sattalite signals, but I do like to call it a sensor. It picks up signals sent by sattalites of different constelations like GPS or Gallileo. It then uses the signals it receives and math to compute where it is on earth. The more sattalites it has the more accurate the calculation is. It outputs a lot of different data, but we primarilly care about about latitude, longitude, sattalite count and fix quality. The latitude is the vertical coordinate on the globe it increases or decreases when you go north or south, like y axis on a graph. The longitude is the horizontal coordinate and changes value when you go west or east, like x axis on a graph. The longitude is a special case, it is not linear like the latitude, it increases in precision when you go further away form the equator. This makes a bit difficult to work with as the code needs to use the linearized version of longitude, but to display the data you need to see it non linearly. The sattalite count is important to know the quality of data that is being delivered, a sattalite number of 10 or above is good quality. The fix type is a lower bar for quality, if the fix type is "3D" it is good enough quality to do some really bad gps position tracking or holding.

The gps can be used for a lot of things, but we are going to use it for position hold and maybe later for waypoint following.

### Barometer
The barometer can tell you how dense the air is. If you lift the drone up it will show lower pressure and if you lower it it will have higher pressure. The drone can sense the change in pressure and then convert that pressure change in to actual measurable change in altitude. This is essential for altitude hold functionality. 

There are problems though, a drone has propellers and propellers can generate wind and low pressure, the barometer can pick up this pressure change and provide fake data that will make the drone think the altitude is changing quicker that expected. Dealing with this is difficult. I have not progreessed far in this area yet, but I have tried to isolate the the sensor using foam and tape so cross wind cannot blow over it and aid can only come in from one side and the result was no improvement. Still some work left to be done

The sensor I chose first was the BMP280, which is a fine sensor and worked fine but I felt that it was not precise enough. I switched it out for the MS5611 which is widely used due to its accuracy. Barometers often come bundled with temperature sensors and even humidity sensors in some cases. These additional sensors aid in altitude calculations as then everything about the atmosphere that is important to altitude can can be sensed.

Calibration for barometers is done in the factory they were made in. The sensors registers hold the calibration data which is read on sensor initialization and applied once data is gotten. The MS5611 sensor is one of those sensors which you have to tell to start taking a data sample of either temperature or pressure and wait a bit until the data is ready. In our case we alternate between getting the temperature and pressure to have the most accurate altitude measurement in the end.

The altitude calculation can be done as an absolute altitude using formula to determine the current altitude from numbers purely. This altitude calculation is often best when setting a initial altitude by manually if it is known. This is good for devices that are intended to measure altitude in a human understandable way. The other options sampling pressure, setting that as the reference pressure and then always measuring the changes in altitude based on how far away it goes off the reference. This is effectively setting the sea level to be at the current pressure. It is useful for applications where you just want to know which way the altitude changed and that it is stable. This second option is what we use for our drone.


## Other hardware 

### A note about powering of hardware 
When i said that the 

### SPI
We talked about I2C before and how it is a protoco for communication, there is another one called

### Debug Radio
To actually communicate to the drone you need something that can pick up wireless signals. When I was making my self-balancing robot I create a remote control that had a radio transceiver(can send and receive radio signals) on it. So i chose to use the same setup on the drone.

A radio module is just like one of the sensors it communicates to the  


### Axies
Explain X Y and Z. 

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




