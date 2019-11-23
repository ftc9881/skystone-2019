package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;

public abstract class TeleOpBase extends LinearOpMode {

    protected Robot robot;
    protected Configuration config;

    protected abstract void initialize();
    protected abstract void update();

    @Override
    public void runOpMode() {
        robot = Robot.newInstance(this);
        config = new Configuration("TeleOp");
        initialize();

        waitForStart();
        while (opModeIsActive()) {
            update();
        }

    }

}
