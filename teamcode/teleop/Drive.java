package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;

/**
 * ExampleDrive is a simple skeleton for a driver controlled OpMode.
 * This contains example usage of driver input, telemetry, and buttons.
 *
 * @author Trinity Chung
 * @version 0.0
 */
@TeleOp(name = "Drive", group = "TeamCode")
//@Disabled
public class Drive extends LinearOpMode {

    private Robot robot;

    private double lfPower = 0;
    private double rfPower = 0;
    private double lbPower = 0;
    private double rbPower = 0;

    private Button slowButton = new Button();


    @Override
    public void runOpMode() {
        try {

            robot = new Robot(this);

            waitForStart();
            while (opModeIsActive()) {

                // Driver Input
                double driveY = gamepad1.right_stick_y;
                double driveX = gamepad1.right_stick_x;
                double driveR= gamepad1.left_stick_x;
                slowButton.update(gamepad1.a);
                double powerFactor = slowButton.is(Button.State.HELD) ? 0.5 : 1.0;

                robot.drive(driveX, driveY, driveR, powerFactor);

                // Telemetry
                telemetry.addData("Power", driveY);
                telemetry.addData("Power Factor", powerFactor);
                telemetry.addData("Bearing (right negative)", robot.imu.getAngularOrientation().firstAngle);
                telemetry.addData("Pitch", robot.imu.getAngularOrientation().secondAngle);
                telemetry.addData("Roll", robot.imu.getAngularOrientation().thirdAngle);

                telemetry.addData("Right Encoder", robot.rightOdometry.getCurrentPosition());
                telemetry.addData("Left Encoder", robot.leftOdometry.getCurrentPosition());
                telemetry.addData("Center Encoder", robot.centerOdometry.getCurrentPosition());

                telemetry.update();
            }

        } catch (RuntimeException e) {

            robot.log(e.getMessage());
        }
    }

}
