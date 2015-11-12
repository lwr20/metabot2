# metabot2
Metabot2 is the Team Metabot (www.hardwarehacker.co.uk) entry for
2015 PiWars (www.piwars.org).

It is a Raspberry Pi powered robot, with an Arduino co-processor.

The code in this repo is split into 4 directories:
 - Arduino
    - contains the code for our Arduino Due, which does all the real-time
    stuff
 - PC
    - contains the code which is run on a separate control PC
    - this reads a joystick and passes joystick data (using JSON over TCP)
        to the metabot.py program running on the robot.
 - RPI
    - contains the code which runs on the Raspberry Pi in the robot itself.

