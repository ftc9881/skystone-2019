package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class BatMobile extends Robot {


    // Singleton pattern; constructor is private and enable only one instance at a time
    private static BatMobile instance;

    public static BatMobile newInstance(LinearOpMode opMode) {
        instance = new BatMobile(opMode);
        return instance;
    }

    public static BatMobile getInstance() {
        return instance;
    }

    public Intake intake;

    private BatMobile(LinearOpMode opMode) {
        super(opMode);

        RobotSystemFactory robotFactory = new RobotSystemFactory(opMode.hardwareMap);
        driveTrain = robotFactory.driveTrain();
        intake = robotFactory.intakeSystem();
    }

}
