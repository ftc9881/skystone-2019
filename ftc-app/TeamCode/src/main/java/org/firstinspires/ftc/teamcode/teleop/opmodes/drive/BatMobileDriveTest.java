package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.servo.ToggleServo;
import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;

import static org.firstinspires.ftc.teamcode.teleop.utility.Button.State.DOUBLE_TAP;
import static org.firstinspires.ftc.teamcode.teleop.utility.Button.State.DOWN;

@TeleOp(group="Drive")
public class BatMobileDriveTest extends BaseDrive {

    private BatMobile batMobile;

    private double deadZone;
    private double slowLiftPowerZone;
    private double extendPowerFactor;
    private double turtleDrivePowerFactor;
    private double snailDrivePowerFactor;
    private double outtakePowerFactor;

    private Button pivotButton = new Button();
    private Button clawButton = new Button();
    private Button foundationButton = new Button();
    private Button capstoneButton = new Button();
    private Button depositLeftButton = new Button();
    private Button depositRightButton = new Button();

    private boolean wasInputtingFullDown = false;

    enum LiftState {
        RUN_TO_POSITION,
        HOLD,
        COAST,
        MANUAL
    }
    private LiftState state = LiftState.COAST;

    @Override
    protected void initialize() {
        super.initialize();
        batMobile = BatMobile.createInstance();
        batMobile.elevator.left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        batMobile.elevator.right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        deadZone = config.getDouble("dead zone", 0.1);
        extendPowerFactor = config.getDouble("extend power", 1.0);
        turtleDrivePowerFactor = config.getDouble("turtle drive power", 0.5);
        snailDrivePowerFactor = config.getDouble("snail drive power", 0.25);
        slowLiftPowerZone = config.getDouble("slow lift zone", 0.7);
        outtakePowerFactor = config.getDouble("outtake power", 1.0);

        batMobile.sideArm.pivot.set(ToggleServo.State.CLOSED);
    }

    @Override
    protected void update() {
        updateDrivePower();
        updateDrive();

        updateButtons();
        updateElevator();
        updateIntake();
        updateServos();

        updateTelemetry();
    }

    private void updateDrivePower() {
        if (gamepad1.right_bumper) {
            drivePowerFactor = snailDrivePowerFactor;
        } else if (gamepad1.left_bumper) {
            drivePowerFactor = turtleDrivePowerFactor;
        } else {
            drivePowerFactor = 1;
        }
    }

    private void updateButtons() {
        pivotButton.update(gamepad1.y);
        clawButton.update(gamepad1.x);
        foundationButton.update(gamepad1.b);
        capstoneButton.update(gamepad2.y);
        depositLeftButton.update(gamepad2.left_bumper);
        depositRightButton.update(gamepad2.right_bumper);
    }

    private void updateElevator() {

        double liftPower = getLiftInput();
        boolean isLifting = isInputting(liftPower);
        boolean isExtending = isInputting(getExtendInputPower());
        boolean isInputting = isLifting || isExtending;
        boolean inputtingFullDown = (int)liftPower == -1;

        if (wasInputtingFullDown && !inputtingFullDown) {
            state = LiftState.COAST;
        } else if (!isInputting && state != LiftState.HOLD && state != LiftState.COAST) {
            state = LiftState.HOLD;
            batMobile.elevator.setRunToRelativePosition(0, 0);
        } else if (isInputting) {
            state = LiftState.MANUAL;
        }

        wasInputtingFullDown = inputtingFullDown;

        switch (state) {
            case HOLD:
            case RUN_TO_POSITION:
                batMobile.elevator.updateRunToRelativePosition();
                break;
            case COAST:
                batMobile.elevator.left.setPower(0);
                batMobile.elevator.right.setPower(0);
                break;
            case MANUAL:
            default:
                updateElevatorManual();
                break;
        }

    }

    private double getLiftInput() {
        double liftPowerP1 = (gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0);
        double liftPowerP2 = Math.sqrt(Math.abs(gamepad2.left_stick_y)) * (-gamepad2.left_stick_y > 0 ? 1 : -1);
        return liftPowerP1 + liftPowerP2;
    }

    private double getLiftInputPower() {
        double liftPowerP1 = (gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0);
        double liftPowerP2 = Math.sqrt(Math.abs(gamepad2.left_stick_y)) * (-gamepad2.left_stick_y > 0 ? 1 : -1);
        if (liftPowerP2 < 0 && liftPowerP2 > -slowLiftPowerZone) {
            liftPowerP2 = 0;
        }
        return liftPowerP1 + liftPowerP2;
    }

    private double getExtendInputPower() {
        double extendPowerP1 = (gamepad1.dpad_right ? 1 : 0) - (gamepad1.dpad_left ? 1 : 0) * extendPowerFactor;
        double extendPowerP2 = gamepad2.right_stick_x;
        return extendPowerP1 + extendPowerP2;
    }

    private void updateElevatorManual() {
//        double powerFactor = isInputting(gamepad2.right_trigger) ? slowLiftPower : 1.0;
        batMobile.elevator.setPowerLE(getLiftInputPower(), getExtendInputPower());
    }

    private boolean isInputting(double input) {
        return Math.abs(input) > deadZone;
    }

    private void updateIntake() {
        double intakePower = (gamepad1.right_trigger - gamepad1.left_trigger) * outtakePowerFactor;
        batMobile.intake.setPower(intakePower);
    }

    private void updateServos() {
        updateToggle(ToggleServo.State.CLOSED, ToggleServo.State.REST, pivotButton, batMobile.sideArm.pivot);
        updateToggle(clawButton, batMobile.sideArm.claw);
        updateCapstone();
        updateFoundationServos();
        updateDeposit();
    }

    private void updateToggle(Button button, ToggleServo ... servos) {
        updateToggle(ToggleServo.State.CLOSED, ToggleServo.State.OPEN, button, servos);
    }

    private void updateToggle(ToggleServo.State stateA, ToggleServo.State stateB, Button button, ToggleServo ... servos) {
        if (button.is(DOWN)) {
            for (ToggleServo servo : servos) {
                servo.toggle(stateA, stateB);
            }
        }
    }

    private void updateDeposit() {
        if (depositLeftButton.is(DOWN)) {
            batMobile.depositServo.set(ToggleServo.State.OPEN);
        }
        if (depositRightButton.is(DOWN)) {
            batMobile.depositServo.set(ToggleServo.State.CLOSED);
        }
    }

    private void updateCapstone() {
        if (capstoneButton.is(DOWN)) {
            batMobile.depositServo.set(ToggleServo.State.REST);
        }
    }

    private void updateFoundationServos() {
        updateToggle(foundationButton, batMobile.leftFoundationServo, batMobile.rightFoundationServo);
        if (foundationButton.is(DOUBLE_TAP)) {
            batMobile.leftFoundationServo.set(ToggleServo.State.REST);
            batMobile.rightFoundationServo.set(ToggleServo.State.REST);
        }
    }


    private void updateTelemetry() {
        telemetry.addData("Lift State", state.name());

        telemetry.addData("Left Lift Position", batMobile.elevator.left.getCurrentPosition());
        telemetry.addData("Right Lift Position", batMobile.elevator.right.getCurrentPosition());
//        telemetry.addData("Lift Position Difference", batMobile.elevator.getClicksDifference());
//        telemetry.addData("Left Lift Velocity", batMobile.elevator.left.getVelocity());
//        telemetry.addData("Right Lift Velocity", batMobile.elevator.left.getVelocity());

        telemetry.addData("Lift Power", getLiftInputPower());
        telemetry.addData("P2 Left Stick Y", gamepad2.left_stick_y);

        telemetry.addData("Drive Power Factor", drivePowerFactor);
        telemetry.addData("Intake Power Factor", outtakePowerFactor);
        telemetry.update();
    }

}
