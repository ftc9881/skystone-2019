package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.hardware.servo.ToggleServo;
import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;

import static org.firstinspires.ftc.teamcode.teleop.utility.Button.State.DOUBLE_TAP;
import static org.firstinspires.ftc.teamcode.teleop.utility.Button.State.DOWN;
import static org.firstinspires.ftc.teamcode.teleop.utility.Button.State.HELD;

@TeleOp(group="Test")
public class BatMobileDriveTest extends BaseDrive {

    private BatMobile batMobile;

    private double deadZone;
    private double liftPowerFactor;
    private double extendPowerFactor;
    private double turtleDrivePowerFactor;
    private double snailDrivePowerFactor;
    private double slowLiftPowerFactor;
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

    private boolean holdingPosition = false;


    @Override
    protected void initialize() {
        super.initialize();
        batMobile = BatMobile.createInstance();

        deadZone = config.getDouble("dead zone", 0.1);
        liftPowerFactor = config.getDouble("lift power", 1.0);
        extendPowerFactor = config.getDouble("extend power", 1.0);
        turtleDrivePowerFactor = config.getDouble("turtle drive power", 0.5);
        snailDrivePowerFactor = config.getDouble("snail drive power", 0.25);
        slowLiftPowerFactor = config.getDouble("slow lift power", 0.3);
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
        if (increaseLiftLevelButton.is(DOWN)) {
            double p = batMobile.elevator.left.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p;
            batMobile.elevator.left.setVelocityPIDFCoefficients(p + 0.5, 3, 0, 0);
            batMobile.elevator.right.setVelocityPIDFCoefficients(p + 0.5, 3, 0, 0);
        }
        if (decreaseLiftLevelButton.is(DOWN)) {
            double p = batMobile.elevator.left.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p;
            batMobile.elevator.left.setVelocityPIDFCoefficients(p - 0.5, 3, 0, 0);
            batMobile.elevator.right.setVelocityPIDFCoefficients(p - 0.5, 3, 0, 0);
        }

        boolean holdLift = isInputting(gamepad2.left_trigger);
        if (holdLift && !holdingPosition) {
            holdingPosition = true;
            batMobile.elevator.runToRelativePosition(0, 0);
            AutoRunner.log("BatMobileDriveTest", "Set position");
        }
        if (!holdLift) {
            holdingPosition = false;
            updateElevatorManual();
        }
//        holdingPosition = isInputting(gamepad2.left_trigger);
//        if (holdingPosition) {
//            batMobile.elevator.setVelocity(0, 0);
//        } else {
//            updateElevatorManual();
//        }
    }

    private double getLiftInputPower() {
        double liftPowerP1 = (gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0) * liftPowerFactor;
        double liftPowerP2 = Math.pow(-gamepad2.left_stick_y, 3);
        return liftPowerP1 + liftPowerP2;
    }

    private double getExtendInputPower() {
        double extendPowerP1 = (gamepad1.dpad_right ? 1 : 0) - (gamepad1.dpad_left ? 1 : 0) * extendPowerFactor;
        double extendPowerP2 = Math.pow(gamepad2.right_stick_x, 3);
        return extendPowerP1 + extendPowerP2;
    }

    private void updateElevatorManual() {
        double powerFactor = isInputting(gamepad2.right_trigger) ? slowLiftPowerFactor : 1.0;
        batMobile.elevator.setPowerLE(getLiftInputPower(), getExtendInputPower(), powerFactor);
    }

    private boolean isInputting(double input) {
        return Math.abs(input) > deadZone;
    }

    private void updateIntake() {
        double intakePower = (gamepad1.right_trigger - gamepad1.left_trigger) * outtakePowerFactor;
        batMobile.intake.left.setPower(intakePower);
        batMobile.intake.right.setPower(-intakePower);
    }

    private void updateServos() {
        updateToggle(ToggleServo.State.CLOSED, ToggleServo.State.REST, pivotButton, batMobile.sideArm.pivot);
        updateToggle(clawButton, batMobile.sideArm.claw);
        updateCapstone();
        updateFoundationServos();
        updateDepositServos();
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

    private void updateCapstone() {
        if (capstoneButton.is(DOWN)) {
            batMobile.backDepositServo.set(ToggleServo.State.REST);
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
        telemetry.addData("Lift kP", batMobile.elevator.left.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p);
        telemetry.addData("Target Pos", batMobile.elevator.left.getTargetPosition());
        telemetry.addData("Mode", batMobile.elevator.left.getMode());
        telemetry.addData("Left Lift Position", batMobile.elevator.left.getCurrentPosition());
        telemetry.addData("Right Lift Position", batMobile.elevator.right.getCurrentPosition());
        telemetry.addData("Running to position", holdingPosition);
        telemetry.addData("Drive Power Factor", drivePowerFactor);
        telemetry.addData("Lift Power Factor", liftPowerFactor);
        telemetry.addData("Intake Power Factor", outtakePowerFactor);
        telemetry.update();
    }

}
