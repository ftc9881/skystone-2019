package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Debug Drive", group = "Debug")
public class DebugDrive extends SimpleDrive {

    @Override
    protected void initialize() {
        super.initialize();
    }

    @Override
    protected void update() {
        driveUsingInput();
        updateIntake();
        updateArm();
        updateTelemetry();
    }

    private void updateIntake() {
        double intakePower = gamepad1.left_trigger - gamepad1.right_trigger * 0.4;
        robot.intake.left.setPower(intakePower);
        robot.intake.right.setPower(intakePower);
    }

    private void updateArm() {
        double liftPower = (gamepad1.dpad_up ? 1.0 : 0.2) + (gamepad1.dpad_down ? -1.0 : 0);
        double swivelPower = (gamepad1.dpad_left ? -0.2 : 0) + (gamepad1.dpad_right ? 0.2 : 0);

        robot.arm.swivelMotor.setPower(swivelPower);
        robot.arm.swivelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.arm.liftMotor.setPower(-liftPower);
        robot.arm.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void updateTelemetry() {

        telemetry.addData("LF Position", robot.driveTrain.lf.getCurrentPosition());
        telemetry.addData("RF Position", robot.driveTrain.rf.getCurrentPosition());
        telemetry.addData("LB Position", robot.driveTrain.lb.getCurrentPosition());
        telemetry.addData("RB Position", robot.driveTrain.rb.getCurrentPosition());

        telemetry.addData("Swivel", robot.arm.swivelMotor.getCurrentPosition());
        telemetry.addData("Lift", robot.arm.liftMotor.getCurrentPosition());

        telemetry.addData("IMU Heading", robot.getImuHeading());

        telemetry.addData("Right Encoder", robot.odometrySystem.getRightPosition());
        telemetry.addData("Left Encoder", robot.odometrySystem.getLeftPosition());
        telemetry.addData("Center Encoder", robot.odometrySystem.getCenterPosition());
        telemetry.addData("Odometry Pose", robot.odometrySystem.getPose().toString());

        telemetry.update();
    }

}
