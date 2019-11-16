package org.firstinspires.ftc.teamcode.teleop.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.teleop.TeleOpBase;

@TeleOp(name = "Simple Drive", group = "TeamCode")
@Disabled
public class SimpleDrive extends TeleOpBase {

    protected Pose drivePose;
    protected double drivePowerFactor;

    @Override
    protected void initialize() {
        drivePose = new Pose(0, 0, 0);
        drivePowerFactor = 1.0;
    }

    @Override
    protected void update() {
        driveUsingInput();
    }

    protected void driveUsingInput() {
        // x and y are reversed because y is forwards
        // up on the gamepad stick is negative
        drivePose.x = -Math.pow(gamepad1.left_stick_y, 3);
        drivePose.y = Math.pow(gamepad1.left_stick_x, 3);
        drivePose.r = Math.pow(gamepad1.right_stick_x, 3);
        robot.driveTrain.drive(drivePose, drivePowerFactor);
    }

}
