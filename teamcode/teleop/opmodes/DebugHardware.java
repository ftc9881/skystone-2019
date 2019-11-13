package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.TeleOpBase;

@TeleOp(name = "Debug Hardware", group = "Debug")
@Disabled
public class DebugHardware extends TeleOpBase {

    @Override
    protected void initialize() {

    }

    @Override
    protected void update() {
        setWheelPowerOnButton();
        updateArm();
        updateIntake();
        updateTelemetry();
    }


    private void setWheelPowerOnButton() {
        robot.driveTrain.lf.setPower(gamepad1.y ? 0.5 : 0);
        robot.driveTrain.rf.setPower(gamepad1.b ? 0.5 : 0);
        robot.driveTrain.lb.setPower(gamepad1.x ? 0.5 : 0);
        robot.driveTrain.rb.setPower(gamepad1.a ? 0.5 : 0);
    }

    private void updateArm() {
        robot.arm.swivelMotor.setPower(gamepad1.left_bumper ? 0.5 : 0);
        robot.arm.swivelMotor.setPower(gamepad1.right_bumper ? -0.5 : 0);

        robot.arm.liftMotor.setPower(gamepad1.right_trigger);
        robot.arm.liftMotor.setPower(-gamepad1.left_trigger);

    }

    private void updateIntake() {
        robot.intake.left.setPower(gamepad1.dpad_up ? 1.0 : 0);
        robot.intake.right.setPower(gamepad1.dpad_up ? 1.0 : 0);

        robot.intake.left.setPower(gamepad1.dpad_down ? -1.0 : 0);
        robot.intake.right.setPower(gamepad1.dpad_down ? -1.0 : 0);
    }

    private void updateTelemetry() {
        telemetry.addData("LF Position", robot.driveTrain.lf.getCurrentPosition());
        telemetry.addData("RF Position", robot.driveTrain.rf.getCurrentPosition());
        telemetry.addData("LB Position", robot.driveTrain.lb.getCurrentPosition());
        telemetry.addData("RB Position", robot.driveTrain.rb.getCurrentPosition());

        telemetry.addData("Swivel", robot.arm.swivelMotor.getCurrentPosition());
        telemetry.addData("Lift", robot.arm.liftMotor.getCurrentPosition());

        telemetry.addData("IMU Heading", robot.getImuHeading());

        telemetry.update();
    }

}
