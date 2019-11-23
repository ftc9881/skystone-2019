package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.teleop.utility.Button;

@TeleOp(name = "Meet 1 Drive", group = "TeleOp")
public class Meet1Drive extends BaseDrive {

    private double deadZone;
    private double slowModePowerFactor;

    private double outtakePowerFactor;


    // TODO: Add button updater helper class
    private Button foundationGrabButton = new Button();

    @Override
    protected void initialize() {
        super.initialize();

        deadZone = config.getDouble("dead zone", 0.1);
        slowModePowerFactor = config.getDouble("slow factor", 0.5);

        outtakePowerFactor = config.getDouble("outtake power", 0.4);
    }

    @Override
    protected void update() {
        drivePowerFactor = gamepad1.left_bumper || gamepad1.right_bumper ? slowModePowerFactor : 1.0;
        super.update();

        foundationGrabButton.update(bothGamepads.b);

        updateFoundationGrabToggle();
        updateSwivelMove();
        updateLiftMove();
        intakeFromInput(bothGamepads);

        updateTelemetry();
    }

    private void updateFoundationGrabToggle() {
        if (foundationGrabButton.is(Button.State.DOWN)) {
            robot.foundationGrabber.toggle();
        }
    }

    private void intakeFromInput(Gamepad gamepad) {
        double intakePower = gamepad.left_trigger - gamepad.right_trigger * outtakePowerFactor;
        robot.intake.left.setPower(intakePower);
        robot.intake.right.setPower(intakePower);
    }

    private void updateSwivelMove() {
        double playerOneInput = (gamepad1.dpad_right ? 1 : 0) - (gamepad1.dpad_left ? 1 : 0);
        double playerTwoInput = gamepad2.right_stick_x;
        // Player One's input overrides Player Two
        double power = Math.abs(playerOneInput) > deadZone ? playerOneInput : playerOneInput + playerTwoInput;
        robot.arm.swivelMotor.setPower(power);
    }

    private void updateLiftMove() {
        boolean playerOneIsInputting = gamepad1.dpad_up || gamepad1.dpad_down;
        boolean playerTwoIsInputting = Math.abs(gamepad2.left_stick_y) > deadZone;

        if (playerOneIsInputting || playerTwoIsInputting) {
            moveLift();
        } else {
            holdLiftPosition();
        }
    }

    private void moveLift() {
        if (robot.arm.liftMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            robot.arm.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        double playerOneInput = (gamepad1.dpad_down ? 1 : 0) - (gamepad1.dpad_up ? 1 : 0);
        double playerTwoInput = gamepad2.left_stick_y;
        robot.arm.liftMotor.setPower(playerOneInput + playerTwoInput);
    }

    private void holdLiftPosition() {
        if (robot.arm.liftMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            int currentPosition = robot.arm.liftMotor.getCurrentPosition();
            robot.arm.liftMotor.setTargetPosition(currentPosition);
            robot.arm.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Left Grab", robot.foundationGrabber.leftServo.getPosition());
        telemetry.addData("Right Grab", robot.foundationGrabber.rightServo.getPosition());

        telemetry.addData("Lift Position", robot.arm.liftMotor.getCurrentPosition());
        telemetry.addData("Swivel Position", robot.arm.swivelMotor.getCurrentPosition());
        telemetry.update();
    }

}
