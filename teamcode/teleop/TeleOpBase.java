package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

public abstract class TeleOpBase extends LinearOpMode {

    protected Robot robot;

    protected abstract void initialize();
    protected abstract void update();

    @Override
    public void runOpMode() {
        robot = new Robot(this);
        initialize();

        waitForStart();
        while (opModeIsActive()) {
            update();
        }

    }

}
