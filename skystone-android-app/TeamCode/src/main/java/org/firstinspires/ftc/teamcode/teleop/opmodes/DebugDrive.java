package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.utility.Button;

@TeleOp(name = "Debug Drive", group = "Debug")
public class DebugDrive extends BaseDrive {

    private Button upButton = new Button();
    private Button downButton = new Button();
    private Button leftButton = new Button();
    private Button rightButton = new Button();

    @Override
    protected void initialize() {
        super.initialize();
    }

    @Override
    protected void update() {
        super.update();

        updateIntake();
        updateArm();

//        updateConfigureServos();
        updateTelemetry();
    }

    private void updateConfigureServos() {
        upButton.update(bothGamepads.dpad_up);
        downButton.update(bothGamepads.dpad_down);
        leftButton.update(bothGamepads.dpad_left);
        rightButton.update(bothGamepads.dpad_right);

        if (upButton.is(Button.State.DOWN)) {
            robot.foundationGrabber.miniGrabServo.setPosition( robot.foundationGrabber.miniGrabServo.getPosition() - 0.1);
        }

        if (downButton.is(Button.State.DOWN)) {
            robot.foundationGrabber.miniGrabServo.setPosition( robot.foundationGrabber.miniGrabServo.getPosition() + 0.1);
        }

        if (leftButton.is(Button.State.DOWN)) {
            robot.foundationGrabber.mainGrabServo.setPosition( robot.foundationGrabber.mainGrabServo.getPosition() - 0.1);
        }

        if (rightButton.is(Button.State.DOWN)) {
            robot.foundationGrabber.mainGrabServo.setPosition( robot.foundationGrabber.mainGrabServo.getPosition() + 0.1);
        }

        telemetry.addData("Main Grab", robot.foundationGrabber.mainGrabServo.getPosition());
        telemetry.addData("Mini Grab", robot.foundationGrabber.miniGrabServo.getPosition());
    }

    private void updateIntake() {
        double intakePower = gamepad1.left_trigger - gamepad1.right_trigger * 0.4;
        robot.intake.left.setPower(intakePower);
        robot.intake.right.setPower(intakePower);
    }

    private void updateArm() {
        double liftPower = (gamepad1.dpad_up ? 1.0 : 0) + (gamepad1.dpad_down ? -1.0 : 0);
        robot.arm.liftMotor.setPower(-liftPower);
        double swivelPower = (gamepad1.dpad_left ? -0.5 : 0) + (gamepad1.dpad_right ? 0.5 : 0);
        robot.arm.swivelMotor.setPower(swivelPower);
    }

    private void updateTelemetry() {


        telemetry.addData("LF Position", robot.driveTrain.lf.getCurrentPosition());
        telemetry.addData("RF Position", robot.driveTrain.rf.getCurrentPosition());
        telemetry.addData("LB Position", robot.driveTrain.lb.getCurrentPosition());
        telemetry.addData("RB Position", robot.driveTrain.rb.getCurrentPosition());

        telemetry.addData("Swivel", robot.arm.swivelMotor.getCurrentPosition());
        telemetry.addData("Lift", robot.arm.liftMotor.getCurrentPosition());

        telemetry.addData("IMU Heading", robot.getImuHeading().getDegrees());

//        telemetry.addData("Right Encoder", robot.odometrySystem.getRightPosition());
//        telemetry.addData("Left Encoder", robot.odometrySystem.getLeftPosition());
//        telemetry.addData("Center Encoder", robot.odometrySystem.getCenterPosition());
//        telemetry.addData("Odometry Pose", robot.odometrySystem.getPose().toString());

        telemetry.update();
    }

}
