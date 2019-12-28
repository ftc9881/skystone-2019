package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class BatMobile extends Robot {

    private BatMobile(LinearOpMode opMode) {
        super(opMode);

        RobotSystemFactory robotFactory = new RobotSystemFactory(opMode.hardwareMap);
//        sensorSystem = robotFactory.sensorSystem();
    }

}
