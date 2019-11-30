# SkyStone
[Game Manual 2](https://www.firstinspires.org/sites/default/files/uploads/resource_library/ftc/game-manual-part-2.pdf)


## About Our Code
The [ftc-app](./ftc-app) folder is the actual android app. You can find our teamcode inside.

* [Configuration System](#configuration-system)
* [Flexible Autonomous System](##flexible-autonomous-system)
* [Modular Robot Structure](##modular-robot-structure)
* [Custom Hardware Devices](##custom-hardware-devices) 


## Configuration System
Testing the PID? Trying to figure out what to put as the distance, or how much the power should be? There's a lot of times in robotics when you have to adjust values. But changing one value in the code and then rebuilding the app and reconnecting the phone takes such a long time. So, we created a configuration system.

On the robot controller phone, we write a [JSON](https://en.wikipedia.org/wiki/JSON) file, with all our values stored. That way, adjusting values is as easy as changing the json text file.

Combined with [wireless debugging](https://blog.jcole.us/2017/04/13/wireless-programming-for-ftc-robots/), we can edit the text file on our computers and instantly run teleop/autonomous without having to unplug the phone or reinstall code.


## Flexible Autonomous System
Our autonomous system is built around the configuration system. In a JSON file, we store a list of commands (MOVE, TURN, etc) with all of their corresponding properties (power, angle, etc). Through AutoRunner, we read each command and execute them in order.

Commands are defined by their name, and we can combine different actions and conditions to create more complex commands. For example, we can combine a Move action and LookForSkystone condition to get the command behavior MoveUntilSkystoneDetected.


## Modular Robot Structure
Since our robot is constantly changing, it is important to be able to quickly adjust code for it. Thus, structured our Robot class to have multiple subsystems (DriveTrain, Arm, etc).


## Custom Hardware Devices
We wrote drivers for a MaxBotix I2CXL Sensor and a Sharp Distance Sensor, since they are more reliable than REV sensors.


# Golden Gears 9881
[FTCScores](https://ftcscores.com/team/9881)

We are a community team based in La Cañada, California. No, we are not Canadian.
Our team was first founded in 2012 as a FIRST Lego League team, working out of the garage. In 2015, we became an FIRST Tech Challenge team, and still making awesome robots in the garage.

![team-picture](./pictures/team.jpg)