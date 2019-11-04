package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;

/**
 * ExampleDrive is a simple skeleton for a driver controlled OpMode.
 * This contains example usage of driver input, telemetry, and buttons.
 *
 * @author Trinity Chung
 * @version 0.0
 */
@TeleOp(name = "Drive Odometry", group = "TeamCode")
//@Disabled
public class DriveVerboseTeleOp extends LinearOpMode {

    private Robot robot;

    private double lfPower = 0;
    private double rfPower = 0;
    private double lbPower = 0;
    private double rbPower = 0;

    private Button slowButton = new Button();

    private Odometry odometry;

    @Override
    public void runOpMode() {
       try {

            robot = new Robot(this);
            odometry = new Odometry(robot);

            waitForStart();
            while (opModeIsActive()) {

                odometry.updatePose();

                // Driver Input
                // x and y are reversed because y is forwards
                double driveY = gamepad1.right_stick_x;
                double driveX = gamepad1.right_stick_y;
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

                telemetry.addData("Sonar", robot.sonarSensor.getDistance());
                telemetry.addData("Sonar Connection", robot.sonarSensor.getConnectionInfo());

                telemetry.addData("Right Encoder", robot.getOdometryRightPosition());
                telemetry.addData("Left Encoder", robot.getOdometryLeftPosition());
                telemetry.addData("Center Encoder", robot.getOdometryCenterPosition());
                telemetry.addData("Odometry Pose", odometry.getPose().toString());

                telemetry.update();
            }

        } catch (RuntimeException e) {

            robot.log(e.getMessage());
        }
    }

}
