package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;

/**
 * ExampleDrive is a simple skeleton for a driver controlled OpMode.
 * This contains example usage of driver input, telemetry, and buttons.
 *
 * @author Trinity Chung
 * @version 0.0
 */
@TeleOp(name = "ExampleDrive", group = "TeamCode")
//@Disabled
public class ExampleDrive extends LinearOpMode {

    private Robot robot;

    private double lfPower = 0;
    private double rfPower = 0;
    private double lbPower = 0;
    private double rbPower = 0;

    private Button button = new Button();

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
                button.update(gamepad1.a);

                // Calculate Power
                lfPower = Range.clip(driveY - driveR + driveX, -1.0, 1.0);
                rfPower = Range.clip(driveY + driveR - driveX, -1.0, 1.0);
                lbPower = Range.clip(driveY - driveR - driveX, -1.0, 1.0);
                rbPower = Range.clip(driveY + driveR + driveX, -1.0, 1.0);

                robot.lf.setPower(lfPower);
                robot.rf.setPower(rfPower);
                robot.lb.setPower(lbPower);
                robot.rb.setPower(rbPower);


                // Telemetry
                telemetry.addData("Power", driveY);
                telemetry.addData("A Button is held", button.is(Button.State.HELD));
                telemetry.addData("Bearing (right negative)", robot.imu.getAngularOrientation().firstAngle);
                telemetry.addData("Pitch", robot.imu.getAngularOrientation().secondAngle);
                telemetry.addData("Roll", robot.imu.getAngularOrientation().thirdAngle);
                telemetry.update();
            }

        } catch (RuntimeException e) {

            robot.log("Error in ExampleDrive", e.getMessage());
        }
    }

}
