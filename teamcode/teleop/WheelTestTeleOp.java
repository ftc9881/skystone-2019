package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * WheelTestTeleOp enables you to control each wheel using gamepad buttons.
 *
 * @author Trinity Chung
 * @version 0.0
 */
@TeleOp(name = "WheelTestTeleOp", group = "TeamCode")
//@Disabled
public class WheelTestTeleOp extends LinearOpMode {

    private Robot robot;

    private double lfPower = 0;
    private double rfPower = 0;
    private double lbPower = 0;
    private double rbPower = 0;


    @Override
    public void runOpMode() {
        try {

            robot = new Robot(this);

            waitForStart();
            while (opModeIsActive()) {
                robot.lf.setPower(gamepad1.y ? 0.5 : 0);
                robot.rf.setPower(gamepad1.b ? 0.5 : 0);
                robot.lb.setPower(gamepad1.x ? 0.5 : 0);
                robot.rb.setPower(gamepad1.a ? 0.5 : 0);

            }

        } catch (RuntimeException e) {

            robot.logTelemetry(e.getMessage());
        }
    }

}
