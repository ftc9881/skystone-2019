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

    private double miniGrabPosition;
    private double mainGrabPosition;
    private double miniReleasePosition;
    private double mainReleasePosition;
    private boolean miniIsGrabbing;
    private boolean mainIsGrabbing;

    // TODO: Add button updater helper class
    private Button miniFoundationGrabButton = new Button();
    private Button mainFoundationGrabButton= new Button();

    @Override
    protected void initialize() {
        super.initialize();

        deadZone = config.getDouble("dead zone", 0.1);
        slowModePowerFactor = config.getDouble("slow factor", 0.5);

        outtakePowerFactor = config.getDouble("outtake power", 0.4);

        miniGrabPosition = config.getDouble("mini grab", 0.7);
        miniReleasePosition = config.getDouble("mini release", 0);
        mainGrabPosition = config.getDouble("main grab", 0.1);
        mainReleasePosition = config.getDouble("main release", 0.8);
    }

    @Override
    protected void update() {
        drivePowerFactor = gamepad1.left_bumper || gamepad1.right_bumper ? slowModePowerFactor : 1.0;
        super.update();

        miniFoundationGrabButton.update(bothGamepads.a);
        mainFoundationGrabButton.update(bothGamepads.b);

        updateFoundationGrabToggle();
        updateSwivelMove();
        updateLiftMove();
        intakeFromInput(bothGamepads);

        updateTelemetry();
    }

    private void updateFoundationGrabToggle() {
        if (miniFoundationGrabButton.is(Button.State.DOWN)) {
            double newPosition = miniIsGrabbing ? miniReleasePosition : miniGrabPosition;
            robot.foundationGrabber.miniGrabServo.setPosition(newPosition);
            miniIsGrabbing = !miniIsGrabbing;
        }
        if (mainFoundationGrabButton.is(Button.State.DOWN)) {
            double newPosition = mainIsGrabbing ? mainReleasePosition : mainGrabPosition;
            robot.foundationGrabber.mainGrabServo.setPosition(newPosition);
            mainIsGrabbing = !mainIsGrabbing;
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
        telemetry.addData("Mini Grab", robot.foundationGrabber.miniGrabServo.getPosition());
        telemetry.addData("Main Grab", robot.foundationGrabber.mainGrabServo.getPosition());

        telemetry.addData("Lift Position", robot.arm.liftMotor.getCurrentPosition());
        telemetry.addData("Swivel Position", robot.arm.swivelMotor.getCurrentPosition());
        telemetry.update();
    }

}
