# TeamCode

This is the teamcode folder that goes inside the [FTC android app sdk](https://github.com/FIRST-Tech-Challenge/SkyStone).

## Robot
The robot class is a singleton which has references to all the devices on the hardware map. All robot movement functions (drive, servo toggle) are in this class.

## Autonomous
```
Autonomous OpMode -> AutoRunner
                         v
                     Read configuration (json) file
                         v
                     Execute every command in configuration
                         |                             |
     Simple actions performed synchronously        Complex actions have Action and EndCondition
                                                       v                             v
                                                   Run main action             Check end condition
                                                                     \      /
                                                   Exit once either action completes or condition is true
                     
```
