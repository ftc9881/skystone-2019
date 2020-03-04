package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp(group="Drive")
@Disabled
public class BatMobileFieldCentricDrive extends BatMobileDrive {

    protected void updateDrive() {
        Pose inputPose = new Pose();
        inputPose.x = Math.pow(gamepad1.left_stick_x, 3);
        inputPose.y = -Math.pow(gamepad1.left_stick_y, 3);
        inputPose.r = Math.pow(gamepad1.right_stick_x, 3);
        telemetry.addData("InputPose", inputPose);
        drivePose = inputPose;
        if (imuInitialized()) {
            double rad = robot.imu.getHeading().getRadians();

            rad += Math.PI/2;
            drivePose.x =  inputPose.y*Math.cos(rad) + inputPose.x*Math.sin(rad);
            drivePose.y = -inputPose.y*Math.sin(rad) + inputPose.x*Math.cos(rad);
        }
        telemetry.addData("DrivePose", drivePose);
        robot.driveTrain.drive(drivePose, drivePowerFactor);
    }

}