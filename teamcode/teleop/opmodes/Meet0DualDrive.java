package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teleop.utility.Button;

@TeleOp(name = "Meet 0 Dual Drive")
public class Meet0DualDrive extends SimpleDrive {

    private static final int LIFT_UP_POSITION = 0;
    private static final int LIFT_DOWN_POSITION = 1000;

    private static final int SWIVEL_IN_POSITION = 0;
    private static final int SWIVEL_OUT_POSITION = 7000;

    private boolean armIsAutoMoving;

    private Button grabButton;
    private Button releaseButton;
    private Button playerOneToggleButton;
    private Button playerTwoToggleButton;
    private boolean grabbed;

    @Override
    protected void initialize() {
        super.initialize();

        grabButton = new Button();
        releaseButton = new Button();
        playerOneToggleButton = new Button();
        playerTwoToggleButton = new Button();
        grabbed = false;
    }

    @Override
    protected void update() {
        drivePowerFactor = 0.8;
        driveUsingInput();
        updateIntake();
        updateAutoArm();
        updateManualArm();

        playerOneToggleButton.update(gamepad1.a);
        playerTwoToggleButton.update(gamepad2.a);
        if (playerOneToggleButton.is(Button.State.DOWN) || playerTwoToggleButton.is(Button.State.DOWN)) {
            grabbed = !grabbed;

            if (grabbed) {
                robot.foundationGrabber.grabServo.setPosition(0.7);
            }
            else {
                robot.foundationGrabber.grabServo.setPosition(0);
            }
        }


//        grabButton.update(gamepad2.x);
//        releaseButton.update(gamepad2.y);

//        if (grabButton.is(Button.State.DOWN)) {
////            double currentPosition = robot.foundationGrabber.grabServo.getPosition();
////            robot.foundationGrabber.grabServo.setPosition(currentPosition + 0.1);
//            robot.foundationGrabber.grabServo.setPosition(0);
//        }
//        if (releaseButton.is(Button.State.DOWN)) {
////            double currentPosition = robot.foundationGrabber.grabServo.getPosition();
////            robot.foundationGrabber.grabServo.setPosition(currentPosition - 0.1);
//            robot.foundationGrabber.grabServo.setPosition(0.7);
//        }

        telemetry.addData("Toggle Servo State", robot.foundationGrabber.grabServo.getPosition());
        telemetry.update();
    }

    private void updateIntake() {

        double playerOneIntakePower = gamepad1.left_trigger - gamepad1.right_trigger * 0.4;
        double playerTwoIntakePower = gamepad2.left_trigger - gamepad2.right_trigger * 0.4;

        double intakePower = playerOneIntakePower + playerTwoIntakePower;

        robot.intake.left.setPower(intakePower);
        robot.intake.right.setPower(intakePower);
    }

    private void updateManualArm() {
        boolean playerTwoIsMoving = Math.abs(gamepad2.right_stick_y) > 0.1;
        if (gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_up || playerTwoIsMoving) {
            armIsAutoMoving = false;
        }

        if (!armIsAutoMoving) {
            manualArmMove();
        }

    }

    private void manualArmMove() {

        double liftPower = (gamepad1.dpad_up ? 1.0 : 0) + (gamepad1.dpad_down? -1.0 : 0) + gamepad2.right_stick_y;
        double swivelPower = (gamepad1.dpad_left ? -1.0 : 0) + (gamepad1.dpad_right ? 1.0 : 0) + gamepad2.right_stick_x;
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
        if (gamepad2.left_bumper || gamepad1.left_bumper) {
            autoArmMoveIn();
        }
        if (gamepad2.right_bumper || gamepad1.right_bumper) {
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
