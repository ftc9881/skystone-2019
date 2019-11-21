package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpConfiguration;

public abstract class TeleOpBase extends LinearOpMode {

    protected Robot robot;
    protected TeleOpConfiguration config;

    protected abstract void initialize();
    protected abstract void update();

    @Override
    public void runOpMode() {
        robot = Robot.newInstance(this);
        config = new TeleOpConfiguration("TeleOp");
        initialize();

        waitForStart();
        while (opModeIsActive()) {
            update();
        }

    }

}
