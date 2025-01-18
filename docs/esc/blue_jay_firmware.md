# How do flash BlueJay without a FC

I used a F3 discovery board that i had from a long time ago, flashed Betaflight on it and used it as a fake flightcontroller that acts as a middleman to the ESC's. I flashed them one at a time from the same pin.

You do not need a F3 discovery. A F4 Discovery or a blackpill F411 with a working USB port will work. For the F3 i had to look at outdated releases to find a firmware that was still compatible with it. F411 is still well supported. I used Esc-configuration site. Worked very good. I used betaflight_4.0.6 for the F3 Discovery

Do not try to use Arduino for this. I tried to use BlHeliSuite16 to flash the firmware but the software kept complaining if i tired to flash BlueJay firmware. It just does not work.

Betaflight guide for F3: https://betaflight.com/docs/wiki/boards/archive/stm32discovery
Guide to Bluejay: https://www.youtube.com/watch?v=yEDhnBUFQNI&t=350s
