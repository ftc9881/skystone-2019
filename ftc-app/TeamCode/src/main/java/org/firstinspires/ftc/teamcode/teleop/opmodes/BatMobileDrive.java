package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.robot.devices.ToggleServo;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;

import static org.firstinspires.ftc.teamcode.teleop.utility.Button.State.DOUBLE_TAP;
import static org.firstinspires.ftc.teamcode.teleop.utility.Button.State.DOWN;
import static org.firstinspires.ftc.teamcode.teleop.utility.Button.State.HELD;

@TeleOp
public class BatMobileDrive extends BaseDrive {

    private BatMobile batMobile;

    private boolean isElevatorAutoMode = false;
    private boolean isLifted = false;
    private boolean isExtended = false;
    private int liftLevel = 0;
    private int extendLevel = 0;

    private double deadZone;
    private double liftPowerFactor;
    private double extendPowerFactor;
    private double slowDrivePowerFactor;
    private double outtakePowerFactor;

    private Button pivotButton = new Button();
    private Button clawButton = new Button();
    private Button foundationButtonP1 = new Button();
    private Button foundationButtonP2 = new Button();
    private Button capstoneButton = new Button();
    private Button depositLeftButton = new Button();
    private Button depositRightButton = new Button();
    private Button increaseLiftLevelButton = new Button();
    private Button decreaseLiftLevelButton = new Button();
    private Button increaseExtendLevelButton = new Button();
    private Button decreaseExtendLevelButton = new Button();
    private Button toggleLiftButton = new Button();
    private Button toggleExtendButton = new Button();

    @Override
    protected void initialize() {
        super.initialize();
        batMobile = BatMobile.getInstance();

        deadZone = config.getDouble("dead zone", 0.1);
        liftPowerFactor = config.getDouble("lift power", 1.0);
        extendPowerFactor = config.getDouble("extend power", 1.0);
        slowDrivePowerFactor = config.getDouble("slow drive", 0.4);
        outtakePowerFactor = config.getDouble("outtake power", 1.0);
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
        drivePowerFactor = gamepad1.left_bumper || gamepad1.right_bumper ? slowDrivePowerFactor : 1.0;
    }

    private void updateButtons() {
        pivotButton.update(gamepad1.y);
        clawButton.update(gamepad1.x);
        foundationButtonP1.update(gamepad1.b);
        foundationButtonP2.update(gamepad2.a);
        capstoneButton.update(gamepad2.y);
        depositLeftButton.update(gamepad2.left_bumper);
        depositRightButton.update(gamepad2.right_bumper);
        increaseLiftLevelButton.update(gamepad2.dpad_up);
        decreaseLiftLevelButton.update(gamepad2.dpad_down);
        increaseExtendLevelButton.update(gamepad2.dpad_right);
        decreaseExtendLevelButton.update(gamepad1.dpad_left);
        toggleExtendButton.update(gamepad2.b);
        toggleLiftButton.update(gamepad2.a);
    }

    private void updateElevator() {
        updateElevatorLevels();
        if (!isElevatorAutoMode && !isManuallyInputtingForElevator()) {
//            holdPosition();
        }
        if (isManuallyInputtingForElevator()) {
            isElevatorAutoMode = false;
        }
        if (isElevatorAutoMode) {
            updateElevatorShortcuts();
        } else {
            updateElevatorManual();
        }
    }

    private void holdPosition() {
        batMobile.elevator.relativeLiftToLevel(0);
    }

    private void updateElevatorLevels() {
        if (increaseLiftLevelButton.is(DOWN)) {
            liftLevel += 1;
        }
        if (decreaseLiftLevelButton.is(DOWN)) {
            liftLevel -= 1;
        }
        if (increaseExtendLevelButton.is(DOWN)) {
            extendLevel += 1;
        }
        if (decreaseExtendLevelButton.is(DOWN)) {
            extendLevel -= 1;
        }
    }

    private boolean isManuallyInputtingForElevator() {
        return gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left ||
                Math.abs(gamepad2.left_stick_y) > deadZone || Math.abs(gamepad2.right_stick_x) > deadZone;
    }

    private void updateElevatorManual() {
        double liftPowerP1 = (gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0) * liftPowerFactor;
        double extendPowerP1 = (gamepad1.dpad_right ? 1 : 0) - (gamepad1.dpad_left ? 1 : 0) * extendPowerFactor;
        double liftPowerP2 = -gamepad2.left_stick_y;
        double extendPowerP2 = gamepad2.right_stick_x;
        batMobile.elevator.setPowerLE(liftPowerP1 + liftPowerP2, extendPowerP1 + extendPowerP2);
    }

    private void updateElevatorShortcuts() {
        if (toggleLiftButton.is(DOWN)) {
            isLifted = !isLifted;
            if (isLifted) {
                liftLevel += 1;
            }
            int level = isLifted ? liftLevel : -liftLevel;
            batMobile.elevator.relativeLiftToLevel(level);
        }
        if (toggleExtendButton.is(DOWN)) {
            isExtended = !isExtended;
            int level = isExtended ? extendLevel : -extendLevel;
            batMobile.elevator.relativeExtendToLevel(level);
        }
    }

    private void updateIntake() {
        double intakePowerP1 = (gamepad1.right_trigger - gamepad1.left_trigger) * outtakePowerFactor;
        double intakePowerP2 = (gamepad2.right_trigger - gamepad2.left_trigger) * outtakePowerFactor;
        double intakePower = intakePowerP1 + intakePowerP2;
        batMobile.intake.setPower(intakePower);
    }

    private void updateServos() {
        updateToggle(capstoneButton, batMobile.capstoneServo);
        updateToggle(pivotButton, batMobile.sideArm.pivot);
        updateToggle(clawButton, batMobile.sideArm.claw);
        updateFoundationServos();
        updateDepositServos();
    }

    private void updateToggle(Button button, ToggleServo ... servos) {
        if (button.is(DOWN)) {
            for (ToggleServo servo : servos) {
                servo.toggle();
            }
        }
    }

    private void updateFoundationServos() {
        updateToggle(foundationButtonP1, batMobile.leftFoundationServo, batMobile.rightFoundationServo);
        updateToggle(foundationButtonP2, batMobile.leftFoundationServo, batMobile.rightFoundationServo);
        if (foundationButtonP1.is(DOUBLE_TAP) || foundationButtonP2.is(DOUBLE_TAP)) {
            batMobile.leftFoundationServo.set(ToggleServo.State.REST);
            batMobile.rightFoundationServo.set(ToggleServo.State.REST);
        }
    }

    private void updateDepositServos() {
        boolean bothPressed = (depositLeftButton.is(DOWN) && depositRightButton.is(HELD)) ||
                              (depositLeftButton.is(HELD) && depositRightButton.is(DOWN)) ||
                              (depositLeftButton.is(DOWN) && depositRightButton.is(DOWN));
        if (bothPressed) {
            batMobile.frontDepositServo.set(ToggleServo.State.CLOSED);
            batMobile.backDepositServo.set(ToggleServo.State.CLOSED);
        }
        else if (depositLeftButton.is(DOWN)) {
            batMobile.frontDepositServo.set(ToggleServo.State.OPEN);
            batMobile.backDepositServo.set(ToggleServo.State.CLOSED);
        } else if (depositRightButton.is(DOWN)) {
            batMobile.frontDepositServo.set(ToggleServo.State.OPEN);
            batMobile.backDepositServo.set(ToggleServo.State.OPEN);
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Left Lift Position", batMobile.elevator.leftLift.motor.getCurrentPosition());
        telemetry.addData("Right Lift Position", batMobile.elevator.rightLift.motor.getCurrentPosition());
        telemetry.addData("Left Lift Power", batMobile.elevator.leftLift.motor.getPower());
        telemetry.addData("Right Lift Power", batMobile.elevator.rightLift.motor.getPower());
        telemetry.addData("Lift Level", liftLevel);
        telemetry.addData("Extend Level", extendLevel);
        telemetry.addData("Drive Power Factor", drivePowerFactor);
        telemetry.addData("Intake Power Factor", outtakePowerFactor);
        telemetry.update();
    }

}
