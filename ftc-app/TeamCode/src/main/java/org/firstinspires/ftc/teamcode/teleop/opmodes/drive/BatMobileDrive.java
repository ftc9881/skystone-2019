package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.servo.ToggleServo;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;

import static org.firstinspires.ftc.teamcode.teleop.utility.Button.State.DOUBLE_TAP;
import static org.firstinspires.ftc.teamcode.teleop.utility.Button.State.DOWN;

@TeleOp(group="Drive")
public class BatMobileDrive extends BaseDrive {

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

    private int clicksPerLiftLevel;

    private boolean holdingPosition = false;
    private boolean runningToPosition = false;
    private boolean holdingWhileExtending = false;
    private boolean coasting = false;
    private boolean wasInputtingFullDown = false;

    enum LiftState {
        RUN_TO_POSITION,
        EXTEND,
        COAST,
        MANUAL
    }

    private LiftState state;

    private double coastDownZone;
    private double minExtendPower;

    private PIDController extendPid;

    private int liftLevel;


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

        coastDownZone = config.getDouble("coast zone", 0.4);
        minExtendPower = config.getDouble("min extend pid power", 0.4);

        clicksPerLiftLevel = config.getInt("clicks per lift level", 300);

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
        updateLevels();

        if (toggleLiftButton.is(DOWN)) {
            runningToPosition = true;
            holdingPosition = false;
            batMobile.elevator.setRunToRelativePosition(clicksPerLiftLevel * liftLevel, 0);
        }

        double liftPower = getLiftInputPower();

        boolean isLifting = isInputting(liftPower);
        boolean isExtending = isInputting(getExtendInputPower());
        boolean isInputting = isLifting || isExtending;

//        coasting = liftPower < 0 && Math.abs(gamepad2.left_stick_y) < coastDownZone && !isExtending;
        boolean inputtingFullDown = (int)liftPower == -1;
        if (wasInputtingFullDown && !inputtingFullDown) {
            coasting = true;
        }
        wasInputtingFullDown = inputtingFullDown;

        if (isExtending || liftPower > 0) {
            coasting = false;
        }

        if (!isInputting && !holdingPosition && !runningToPosition && !coasting) {
            holdingPosition = true;
            batMobile.elevator.setRunToRelativePosition(0, 0);
        }

        if (isInputting) {
            holdingPosition = false;
            runningToPosition = false;
        }

        if (!isExtending || (isLifting && isExtending)) {
            holdingWhileExtending = false;
        }

        if (isExtending && !isLifting) {
            holdingWhileExtending = true;
            extendPid = new PIDController(config, "extend", batMobile.elevator.getClicksDifference());
        }


        if (holdingPosition || runningToPosition) {
            batMobile.elevator.updateRunToRelativePosition();
        } else if (holdingWhileExtending) {
            int difference = batMobile.elevator.getClicksDifference();
            double correctedPower = Range.clip(extendPid.getCorrectedOutput(difference), minExtendPower, 1);
            batMobile.elevator.setPowerLE(correctedPower, getExtendInputPower());
        } else if (coasting) {
            batMobile.elevator.left.setPower(0);
            batMobile.elevator.right.setPower(0);
        } else {
            updateElevatorManual();
        }
    }

    private void updateLevels() {
        if (increaseLiftLevelButton.is(DOWN)) {
            liftLevel += 1;
        }
        if (decreaseLiftLevelButton.is(DOWN)) {
            liftLevel -= 1;
        }
    }

    private double getLiftInputPower() {
        double liftPowerP1 = (gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0) * liftPowerFactor;
//        double liftPowerP2 = Math.pow(-gamepad2.left_stick_y, 3);
        double liftPowerP2 = -gamepad2.left_stick_y;
        return liftPowerP1 + liftPowerP2;
    }

    private double getExtendInputPower() {
        double extendPowerP1 = (gamepad1.dpad_right ? 1 : 0) - (gamepad1.dpad_left ? 1 : 0) * extendPowerFactor;
//        double extendPowerP2 = Math.pow(gamepad2.right_stick_x, 3);
        double extendPowerP2 = gamepad2.right_stick_x;
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
//        batMobile.intake.left.setPower(intakePower);
//        batMobile.intake.right.setPower(-intakePower);
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
        updateToggle(foundationButtonP1, batMobile.leftFoundationServo, batMobile.rightFoundationServo);
        updateToggle(foundationButtonP2, batMobile.leftFoundationServo, batMobile.rightFoundationServo);
        if (foundationButtonP1.is(DOUBLE_TAP) || foundationButtonP2.is(DOUBLE_TAP)) {
            batMobile.leftFoundationServo.set(ToggleServo.State.REST);
            batMobile.rightFoundationServo.set(ToggleServo.State.REST);
        }
    }


    private void updateTelemetry() {
        telemetry.addData("Lift Level", liftLevel);
        telemetry.addData("Target Pos", batMobile.elevator.left.getTargetPosition());

        telemetry.addData("Running to position", runningToPosition);
        telemetry.addData("Holding position", holdingPosition);
        telemetry.addData("Coasting", coasting);

        telemetry.addData("Left Lift Position", batMobile.elevator.left.getCurrentPosition());
        telemetry.addData("Right Lift Position", batMobile.elevator.right.getCurrentPosition());
        telemetry.addData("Lift Position Difference", batMobile.elevator.getClicksDifference());
        telemetry.addData("Left Lift Velocity", batMobile.elevator.left.getVelocity());
        telemetry.addData("Right Lift Velocity", batMobile.elevator.left.getVelocity());

        telemetry.addData("Lift Power", getLiftInputPower());
        telemetry.addData("P2 Left Stick Y", gamepad2.left_stick_y);

        telemetry.addData("Drive Power Factor", drivePowerFactor);
        telemetry.addData("Lift Power Factor", liftPowerFactor);
        telemetry.addData("Intake Power Factor", outtakePowerFactor);
        telemetry.update();
    }

}
