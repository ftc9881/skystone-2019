package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.TeleOpBase;

@TeleOp(name = "Debug Hardware", group = "Debug")
public class DebugHardware extends TeleOpBase {

    @Override
    protected void initialize() {

    }

    @Override
    protected void update() {
        setWheelPowerOnButton();
        updateTelemetry();
    }


    private void setWheelPowerOnButton() {
        robot.driveTrain.lf.setPower(gamepad1.y ? 0.5 : 0);
        robot.driveTrain.rf.setPower(gamepad1.b ? 0.5 : 0);
        robot.driveTrain.lb.setPower(gamepad1.x ? 0.5 : 0);
        robot.driveTrain.rb.setPower(gamepad1.a ? 0.5 : 0);
    }

    private void updateTelemetry() {
        telemetry.addData("LF Position", robot.driveTrain.lf.getCurrentPosition());
        telemetry.addData("RF Position", robot.driveTrain.rf.getCurrentPosition());
        telemetry.addData("LB Position", robot.driveTrain.lb.getCurrentPosition());
        telemetry.addData("RB Position", robot.driveTrain.rb.getCurrentPosition());

        telemetry.addData("Bearing (right negative)", robot.imu.getAngularOrientation().firstAngle);
        telemetry.addData("Pitch", robot.imu.getAngularOrientation().secondAngle);
        telemetry.addData("Roll", robot.imu.getAngularOrientation().thirdAngle);

        telemetry.addData("Sonar", robot.sonarSensor.getDistance());
        telemetry.addData("Sonar Connection", robot.sonarSensor.getConnectionInfo());

        telemetry.update();
    }

}
