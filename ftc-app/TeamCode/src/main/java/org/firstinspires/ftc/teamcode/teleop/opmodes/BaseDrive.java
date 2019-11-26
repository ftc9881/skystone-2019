package org.firstinspires.ftc.teamcode.teleop.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.teleop.utility.Axis;
import org.firstinspires.ftc.teamcode.teleop.utility.InputManager.Player;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp(name = "Base Drive", group = "TeamCode")
@Disabled
public class BaseDrive extends TeleOpBase {

    protected double drivePowerFactor;
    private Pose drivePose;

    @Override
    protected void initialize() {
        drivePose = new Pose(0, 0, 0);
        drivePowerFactor = 1.0;

        inputManager
            .addAxis("drive x", Player.ONE, Axis.Input.LEFT_STICK_X)
            .addAxis("drive y", Player.ONE, Axis.Input.LEFT_STICK_Y)
            .addAxis("drive r", Player.ONE, Axis.Input.RIGHT_STICK_X);
    }

    @Override
    protected void update() {
        updateDrive();
    }

    protected void updateDrive() {
        // x and y are reversed because y is forwards
        // up on the gamepad stick is negative
        drivePose.y = Math.pow(inputManager.getAxisValue("drive x"), 3);
        drivePose.x = -Math.pow(inputManager.getAxisValue("drive y"), 3);
        drivePose.r = -Math.pow(inputManager.getAxisValue("drive r"), 3);
        robot.driveTrain.drive(drivePose, drivePowerFactor);
    }

}