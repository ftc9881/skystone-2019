package org.firstinspires.ftc.teamcode.teleop.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.TeleOpBase;

@TeleOp(name = "Simple Drive", group = "TeamCode")
@Disabled
public class SimpleDrive extends TeleOpBase {

    protected double driveY;
    protected double driveX;
    protected double driveR;
    protected double drivePowerFactor;

    @Override
    protected void initialize() {
        drivePowerFactor = 1.0;
    }

    @Override
    protected void update() {
        driveUsingInput();
    }

    protected void driveUsingInput() {
        // x and y are reversed because y is forwards
        driveY = Math.pow(gamepad1.left_stick_x, 3);
        driveX = -Math.pow(gamepad1.left_stick_y, 3);
        driveR = -Math.pow(gamepad1.right_stick_x, 3);
        robot.drive(driveX, driveY, driveR, drivePowerFactor);
    }

}
