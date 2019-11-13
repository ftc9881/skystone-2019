package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teleop.utility.Button;

@TeleOp(name = "Meet 0")
public class Meet0Drive extends SimpleDrive {

    private static final int LIFT_UP_POSITION = 0;
    private static final int LIFT_DOWN_POSITION = 1000;

    private static final int SWIVEL_IN_POSITION = 0;
    private static final int SWIVEL_OUT_POSITION = 7000;

    private boolean armIsAutoMoving;

    private Button grabButton;
    private Button releaseButton;

    @Override
    protected void initialize() {
        super.initialize();

        grabButton = new Button();
        releaseButton = new Button();
    }

    @Override
    protected void update() {
        drivePowerFactor = 0.8;
        driveUsingInput();
        updateIntake();
        updateAutoArm();
        updateManualArm();

        grabButton.update(gamepad1.x);
        releaseButton.update(gamepad1.y);

        if (grabButton.is(Button.State.DOWN)) {
//            double currentPosition = robot.foundationGrabber.grabServo.getPosition();
//            robot.foundationGrabber.grabServo.setPosition(currentPosition + 0.1);
            robot.foundationGrabber.grabServo.setPosition(0);
        }
        if (releaseButton.is(Button.State.DOWN)) {
//            double currentPosition = robot.foundationGrabber.grabServo.getPosition();
//            robot.foundationGrabber.grabServo.setPosition(currentPosition - 0.1);
            robot.foundationGrabber.grabServo.setPosition(0.7);
        }
        telemetry.addData("Grab Servo", robot.foundationGrabber.grabServo.getPosition());
        telemetry.update();
    }

    private void updateIntake() {
        double intakePower = gamepad1.left_trigger - gamepad1.right_trigger * 0.4;
        robot.intake.left.setPower(intakePower);
        robot.intake.right.setPower(intakePower);
    }

    private void updateManualArm() {
        if (gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_up) {
            armIsAutoMoving = false;
        }

        if (!armIsAutoMoving) {
            manualArmMove();
        }

    }

    private void manualArmMove() {

        double liftPower = (gamepad1.dpad_up ? 1.0 : 0) + (gamepad1.dpad_down? -1.0 : 0);
        double swivelPower = (gamepad1.dpad_left ? -1.0 : 0) + (gamepad1.dpad_right ? 1.0 : 0);
        double swivelPosition = robot.arm.swivelMotor.getCurrentPosition();

        // Commented to disable limits for now
//        if (swivelPosition > SWIVEL_OUT_POSITION)  {
//            swivelPower = Math.min(swivelPower, 0);
//        }
//        if (swivelPosition < SWIVEL_IN_POSITION) {
//            swivelPower = Math.max(swivelPower, 0);
//        }
        robot.arm.swivelMotor.setPower(swivelPower);
        robot.arm.swivelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.arm.liftMotor.setPower(-liftPower);
        robot.arm.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void updateAutoArm() {
        if (gamepad1.left_bumper) {
            autoArmMoveIn();
        }
        if (gamepad1.right_bumper) {
            autoArmMoveOut();
        }
   }

    private void autoArmMoveOut() {
        armIsAutoMoving = true;

        int swivelTargetPosition = SWIVEL_OUT_POSITION;
        robot.arm.swivelMotor.setTargetPosition(swivelTargetPosition);
        robot.arm.swivelMotor.setPower(0.5);

        int liftTargetPosition = LIFT_UP_POSITION;
        robot.arm.swivelMotor.setTargetPosition(liftTargetPosition);
        robot.arm.swivelMotor.setPower(0.5);
    }

    private void autoArmMoveIn() {
        armIsAutoMoving = true;

        int swivelTargetPosition = SWIVEL_IN_POSITION;
        robot.arm.swivelMotor.setTargetPosition(swivelTargetPosition);
        robot.arm.swivelMotor.setPower(-0.5);
        robot.arm.swivelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int liftTargetPosition = LIFT_DOWN_POSITION;
        robot.arm.liftMotor.setTargetPosition(liftTargetPosition);
        robot.arm.liftMotor.setPower(-0.5);
        robot.arm.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
