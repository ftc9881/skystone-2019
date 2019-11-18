package org.firstinspires.ftc.teamcode.teleop.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.teleop.TeleOpBase;
import org.firstinspires.ftc.teamcode.teleop.utility.GamepadCombiner;

@TeleOp(name = "Base Drive", group = "TeamCode")
@Disabled
public class BaseDrive extends TeleOpBase {

    protected GamepadCombiner gamepadCombiner;
    protected Gamepad bothGamepads;

    protected double drivePowerFactor;
    private Pose drivePose;

    @Override
    protected void initialize() {
        gamepadCombiner = new GamepadCombiner();
        gamepadCombiner.add(gamepad1).add(gamepad2);

        drivePose = new Pose(0, 0, 0);
        drivePowerFactor = 1.0;
    }

    @Override
    protected void update() {
        bothGamepads = gamepadCombiner.getCombinedGamepadOutput();
        driveFromInput(gamepad1);
    }

    protected void driveFromInput(Gamepad gamepad) {
        // x and y are reversed because y is forwards
        // up on the gamepad stick is negative
        drivePose.x = -Math.pow(gamepad.left_stick_y, 3);
        drivePose.y = Math.pow(gamepad.left_stick_x, 3);
        drivePose.r = Math.pow(gamepad.right_stick_x, 3);
        robot.driveTrain.drive(drivePose, drivePowerFactor);
    }

}
