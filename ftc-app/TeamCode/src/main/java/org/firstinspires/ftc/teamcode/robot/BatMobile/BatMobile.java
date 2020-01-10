package org.firstinspires.ftc.teamcode.robot.BatMobile;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotSystemFactory;

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
    public SideArm sideArm;

    private BatMobile(LinearOpMode opMode) {
        super(opMode);

        sideArm = new SideArm(hardwareMap);

        RobotSystemFactory robotFactory = new RobotSystemFactory(hardwareMap);
        driveTrain = robotFactory.driveTrain();
        intake = robotFactory.intakeSystem();
    }

}
