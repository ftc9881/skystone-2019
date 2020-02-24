package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.hardware.servo.ToggleServo;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;

import static org.firstinspires.ftc.teamcode.teleop.utility.Button.State.DOUBLE_TAP;
import static org.firstinspires.ftc.teamcode.teleop.utility.Button.State.DOWN;

@TeleOp(group="Drive")
public class BatMobileDriveNew extends BaseDrive {

    private BatMobile batMobile;

    private double deadZone;
    private double slowLiftPowerZone;
    private double slowLiftPowerFactor;
    private double liftPowerFactor;
    private double extendPowerFactor;
    private double lbDrivePower;
    private double rbDrivePower;
    private double defaultDrivePower;
    private double outtakePowerFactor;

    private Button pivotButton = new Button();
    private Button clawButton = new Button();
    private Button foundationButton = new Button();
    private Button capstoneButton = new Button();
    private Button depositButton = new Button();
    private Button anglePIDButton = new Button();

    private boolean wasInputtingFullDown = false;
    private boolean liftIsUp = false;

    enum LiftState {
        RUN_TO_POSITION,
        HOLD,
        COAST,
        MANUAL
    }
    private LiftState state = LiftState.COAST;

    private PIDController anglePID;
    private PIDCoefficients anglePIDCoefficients;
    private boolean useAnglePID;

    @Override
    protected void initialize() {
        super.initialize();
        batMobile = BatMobile.createInstance();

        deadZone = config.getDouble("dead zone", 0.1);
        liftPowerFactor = config.getDouble("lift power", 1.0);
        extendPowerFactor = config.getDouble("extend power", 1.0);
        lbDrivePower = config.getDouble("lb drive power", 0.5);
        rbDrivePower = config.getDouble("rb drive power", 0.25);
        defaultDrivePower = config.getDouble("default drive power", 0.25);
        slowLiftPowerFactor = config.getDouble("slow lift power", 0.3);
        slowLiftPowerZone = config.getDouble("slow lift zone", 0.7);
        outtakePowerFactor = config.getDouble("outtake power", 1.0);
        anglePIDCoefficients = config.getKPID("angle");

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

        updatePID();

        updateTelemetry();
    }

    private void updateDrivePower() {
        if (gamepad1.right_bumper || liftIsUp) {
            drivePowerFactor = rbDrivePower;
        } else if (gamepad1.left_bumper) {
            drivePowerFactor = lbDrivePower;
        } else {
            drivePowerFactor = defaultDrivePower;
        }
    }

    private void updateButtons() {
        anglePIDButton.update(gamepad1.a);
        pivotButton.update(gamepad1.y);
        clawButton.update(gamepad1.x);
        foundationButton.update(gamepad1.b);
        capstoneButton.update(gamepad2.y);
        depositButton.update(gamepad2.left_bumper);
    }

    private void updateElevator() {
        double liftPower = getLiftInputPower();
        boolean isLifting = isInputting(liftPower);
        boolean isExtending = isInputting(getExtendInputPower());
        boolean isInputting = isLifting || isExtending;
        boolean inputtingFullDown = (int)liftPower == -1;
        if (liftPower > 0.1) {
            liftIsUp = true;
        } else if (inputtingFullDown) {
            liftIsUp = false;
        }

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

    private double getLiftInputPower() {
        double liftPowerP1 = (gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0) * liftPowerFactor;
        double liftPowerP2 = Math.sqrt(Math.abs(gamepad2.left_stick_y)) * (-gamepad2.left_stick_y > 0 ? 1 : -1);
        if (liftPowerP2 < 0 && liftPowerP2 > -slowLiftPowerZone) {
            liftPowerP2 *= slowLiftPowerFactor;
        }
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
        double intakePower = (gamepad1.right_trigger - gamepad1.left_trigger);
        // TODO: Right >/< for outtake?
        batMobile.intake.setPower(intakePower * (intakePower < 0 ? outtakePowerFactor : 1));
    }

    private void updateServos() {
        updateToggle(ToggleServo.State.CLOSED, ToggleServo.State.REST, pivotButton, batMobile.sideArm.pivot);
        updateToggle(ToggleServo.State.CLOSED, ToggleServo.State.OPEN, depositButton, batMobile.depositServo);
        updateToggle(clawButton, batMobile.sideArm.claw);
        updateCapstone();
        updateFoundationServos();
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

    private void updatePID() {
        if (anglePIDButton.is(DOWN)) {
            useAnglePID = !useAnglePID;
            if (useAnglePID) {
                anglePID = new PIDController(anglePIDCoefficients, robot.imu.getIntegratedHeading().getDegrees());
            }
        }
    }

    private double getAnglePIDPower() {
        return anglePID.getCorrectedOutput(robot.imu.getIntegratedHeading().getDegrees());
    }

    @Override
    protected void updateDrive() {
        // up on the gamepad stick is negative
        drivePose.x = Math.pow(gamepad1.left_stick_x, 3);
        drivePose.y = -Math.pow(gamepad1.left_stick_y, 3);
        drivePose.r = Math.pow(gamepad1.right_stick_x, 3);
        if (useAnglePID && !isInputting(drivePose.r)) {
            drivePose.r = getAnglePIDPower();
        }
        robot.driveTrain.drive(drivePose, drivePowerFactor);
    }


    private void updateTelemetry() {
        telemetry.addData("Lift State", state.name());
        telemetry.addData("Lift Power", getLiftInputPower());
        telemetry.addData("Drive Power", drivePowerFactor);
        telemetry.addData("Using Angle PID?", useAnglePID);
        telemetry.addData("DriveR", drivePose.r);
        telemetry.update();
    }

}
