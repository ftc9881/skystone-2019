package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class BatMobile extends Robot {

    private BatMobile(OpMode opMode) {
        super(opMode);

        RobotSystemFactory robotFactory = new RobotSystemFactory(opMode.hardwareMap);
//        sensorSystem = robotFactory.sensorSystem();
    }

}
