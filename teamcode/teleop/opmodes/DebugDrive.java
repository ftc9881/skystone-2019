package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.utility.Button;

@TeleOp(name = "Debug Drive", group = "Debug")
@Disabled
public class DebugDrive extends SimpleDrive {

    private Button slowButton;

    @Override
    protected void initialize() {
        super.initialize();
        slowButton = new Button();
    }

    @Override
    protected void update() {
        //TODO: Configure odometry wheels
//        robot.odometrySystem.updatePose();
//        slowButton.update(gamepad1.a);
//        drivePowerFactor = slowButton.is(Button.State.HELD) ? 0.5 : 1.0;
        driveUsingInput();
        updateArm();
        updateIntake();
        updateTelemetry();
    }

    private void updateArm() {
        double swivelOutPower = gamepad1.dpad_left ? 0.5 : 0;
        double swivelInPower = gamepad1.dpad_right ? 0.5 : 0;
        robot.arm.swivelMotor.setPower(swivelOutPower - swivelInPower);

        double liftPower = gamepad1.right_trigger;
        double downPower = gamepad1.left_trigger;
        robot.arm.liftMotor.setPower(liftPower - downPower);

    }

    private void updateIntake() {
        double intakePower = gamepad1.dpad_up ? 1.0 : 0;
        double outtakePower = gamepad1.dpad_down ? 1.0 : 0;

        robot.intake.left.setPower(intakePower - outtakePower);
        robot.intake.right.setPower(intakePower - outtakePower);
    }

    private void updateTelemetry() {
        telemetry.addData("Power", driveY);
        telemetry.addData("Power Factor", drivePowerFactor);

        telemetry.addData("LF Position", robot.driveTrain.lf.getCurrentPosition());
        telemetry.addData("RF Position", robot.driveTrain.rf.getCurrentPosition());
        telemetry.addData("LB Position", robot.driveTrain.lb.getCurrentPosition());
        telemetry.addData("RB Position", robot.driveTrain.rb.getCurrentPosition());

        telemetry.addData("Swivel", robot.arm.swivelMotor.getCurrentPosition());
        telemetry.addData("Lift", robot.arm.liftMotor.getCurrentPosition());

        telemetry.addData("IMU Heading", robot.getImuHeading());

        //TODO: Encoders are not currently connected or configured
//        telemetry.addData("Right Encoder", robot.odometrySystem.getRightPosition());
//        telemetry.addData("Left Encoder", robot.odometrySystem.getLeftPosition());
//        telemetry.addData("Center Encoder", robot.odometrySystem.getCenterPosition());
//        telemetry.addData("Odometry Pose", robot.odometrySystem.getPose().toString());

        telemetry.update();
    }
}
