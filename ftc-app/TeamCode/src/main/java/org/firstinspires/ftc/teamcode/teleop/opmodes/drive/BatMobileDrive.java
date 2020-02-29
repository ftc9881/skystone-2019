package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.SomethingBadHappened;
import org.firstinspires.ftc.teamcode.hardware.servo.ToggleServo;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;

import static org.firstinspires.ftc.teamcode.teleop.utility.Button.State.DOUBLE_TAP;
import static org.firstinspires.ftc.teamcode.teleop.utility.Button.State.DOWN;
import static org.firstinspires.ftc.teamcode.teleop.utility.Button.State.HELD;

@TeleOp(group="Drive")
public class BatMobileDrive extends BaseDrive {

    enum LiftState {
        RUN_TO_POSITION,
        HOLD,
        COAST,
        MANUAL,
        MANUAL_EXTENDING
    }
    private LiftState state = LiftState.COAST;

    private BatMobile batMobile;

    private double deadZone;
    private double slowLiftPowerZone;
    private double slowLiftPowerFactor;
    private double lbDrivePower;
    private double rbDrivePower;
    private double liftIsUpDrivePower;
    private double defaultDrivePower;
    private double outtakePowerFactor;

    private Button pivotButton = new Button();
    private Button clawButton = new Button();
    private Button goToAngleButton = new Button();
    private Button foundationButton = new Button();
    private Button capstoneButton = new Button();
    private Button depositButton = new Button();
    private Button toggleSideButton = new Button();
    private Button toggleDepositUpWhenIntakeButton = new Button();
    private Button setAngleSetpointButton = new Button();
    private Button goToLiftLevelButton = new Button();
    private Button increaseLiftLevelButton = new Button();
    private Button decreaseLiftLevelButton = new Button();

    private int liftLevel = 0;
    private int clicksPerLevel;

    private long timeAtFullDown = 0;
    private double maxStickWindow;
    private double fullDownTimeWindow;
    private boolean liftIsUp = false;
    private boolean liftWasUp = false;

    private InitIMUOnThread initIMUAction;
    private PIDController anglePID;
    private PIDCoefficients anglePIDCoefficients;
    private double degreesSetpoint = 0;
    private boolean goingToAngle = false;
    private boolean toggleDepositUpWhenIntake = true;
    private boolean invertXY;
    private AutoRunner.Side side = AutoRunner.Side.RED;

    private double snapAngleError;


    @Override
    protected void initialize() {
        super.initialize();
        batMobile = BatMobile.createInstance();

        deadZone = config.getDouble("dead zone", 0.1);

        lbDrivePower = config.getDouble("lb drive power", 0.5);
        rbDrivePower = config.getDouble("rb drive power", 0.25);
        liftIsUpDrivePower = config.getDouble("lift up drive power", Math.min(lbDrivePower, rbDrivePower));

        snapAngleError = config.getDouble("snap angle error", 3);

        double m = config.getDouble("m battery drive power", 0.05);
        double b = config.getDouble("b battery drive power", 1.4);
        double batteryBasedDrivePower = -m * robot.getBatteryVoltage() + b;
        defaultDrivePower = config.getDouble("default drive power", batteryBasedDrivePower);

        slowLiftPowerFactor = config.getDouble("slow lift power", 0.3);
        slowLiftPowerZone = config.getDouble("slow lift zone", 0.7);
        outtakePowerFactor = config.getDouble("outtake power", 1.0);
        anglePIDCoefficients = config.getKPID("angle");

        clicksPerLevel = config.getInt("clicks per level", 300);

        maxStickWindow = config.getDouble("max stick window", 0.97);
        fullDownTimeWindow = config.getDouble("full down time window", 300);

//        batMobile.redSideArm.setPivotToInsideRestingPosition();
//        batMobile.blueSideArm.setPivotToInsideRestingPosition();
        batMobile.redSideArm.pivot.set(ToggleServo.State.CLOSED);
        batMobile.blueSideArm.pivot.set(ToggleServo.State.CLOSED);

        initIMUAction = new InitIMUOnThread();
        initIMUAction.start();
    }

    @Override
    protected void update() {
        updateDrivePower();
        updateDrive();

        updateButtons();
        updateLiftLevels();
        updateElevator();
        updateIntake();
        updateServos();
        updatePID();

        updateTelemetry();
    }

    private void updateDrivePower() {
        if (gamepad1.left_bumper) {
            drivePowerFactor = lbDrivePower;
        } else if (gamepad1.right_bumper) {
            drivePowerFactor = liftIsUp ? defaultDrivePower : rbDrivePower;
        } else {
            drivePowerFactor = liftIsUp ? liftIsUpDrivePower : defaultDrivePower;
        }
    }

    private void updateButtons() {
        goToAngleButton.update(gamepad1.left_bumper);
        toggleSideButton.update(gamepad1.dpad_up);
        toggleDepositUpWhenIntakeButton.update(gamepad1.dpad_right);
        setAngleSetpointButton.update(gamepad1.dpad_down);
        pivotButton.update(gamepad1.y);
        clawButton.update(gamepad1.x);
        foundationButton.update(gamepad1.b);
        capstoneButton.update(gamepad2.y);
        goToLiftLevelButton.update(gamepad2.x);
        depositButton.update(gamepad2.right_bumper);
        increaseLiftLevelButton.update(gamepad2.dpad_up);
        decreaseLiftLevelButton.update(gamepad2.dpad_down);
    }

    private void updateLiftLevels() {
        if (increaseLiftLevelButton.is(DOWN)) {
            liftLevel += 1;
        }
        if (decreaseLiftLevelButton.is(DOWN)) {
            liftLevel -= 1;
        }
    }

    private void updateElevator() {

        double liftPower = getLiftInput();
        boolean isLifting = isInputting(liftPower);
        boolean isExtending = isInputting(getExtendInputPower());
        boolean inputtingFullDown = liftPower < -maxStickWindow;
        liftWasUp = liftIsUp;

        if (inputtingFullDown) {
            liftIsUp = false;
            timeAtFullDown = System.currentTimeMillis();
        } else if (liftPower > maxStickWindow) {
            liftIsUp = true;
        }
        long msSinceFullDown = System.currentTimeMillis() - timeAtFullDown;

        if (isExtending && !isLifting) {
            state = LiftState.MANUAL_EXTENDING;
        } else if (isExtending || isLifting) {
            state = LiftState.MANUAL;
        } else if (msSinceFullDown < fullDownTimeWindow && !inputtingFullDown || (state == LiftState.MANUAL_EXTENDING && !liftIsUp)) {
            state = LiftState.COAST;
        } else if (goToLiftLevelButton.is(DOWN)) {
            liftLevel += 1;
            state = LiftState.RUN_TO_POSITION;
            batMobile.elevator.setRunToRelativePosition(liftLevel * clicksPerLevel, 0);
        } else if (state != LiftState.HOLD && state != LiftState.COAST && state != LiftState.RUN_TO_POSITION) {
            state = LiftState.HOLD;
            batMobile.elevator.setRunToRelativePosition(0, 0);
        }

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
            case MANUAL_EXTENDING:
            default:
                updateElevatorManual();
                break;
        }
    }

    private double getLiftInput() {
        return -gamepad2.left_stick_y;
    }

    private double getLiftInputPower() {
        double liftPower = Math.sqrt(Math.abs(gamepad2.left_stick_y)) * (-gamepad2.left_stick_y > 0 ? 1 : -1);
        if (liftPower < 0 && liftPower > -slowLiftPowerZone) {
            liftPower = 0;
        }
        return liftPower;
    }

    private double getExtendInputPower() {
        return gamepad2.right_stick_x + gamepad2.right_stick_y;
    }

    private void updateElevatorManual() {
        double powerFactor = gamepad2.left_bumper ? slowLiftPowerFactor : 1.0;
        batMobile.elevator.setPowerLE(getLiftInputPower(), getExtendInputPower(), powerFactor);
    }

    private boolean isInputting(double input) {
        return Math.abs(input) > deadZone;
    }

    private void updateIntake() {
        if (toggleDepositUpWhenIntakeButton.is(DOWN)) {
            toggleDepositUpWhenIntake = !toggleDepositUpWhenIntake;
        }

        double intakePower = (gamepad1.right_trigger - gamepad1.left_trigger);
        if (intakePower > 0 && toggleDepositUpWhenIntake) {
            batMobile.depositServo.set(ToggleServo.State.OPEN);
        }

        double powerFactor = intakePower < 0 && intakePower > -1 ? outtakePowerFactor : 1;
        batMobile.intake.setPower(intakePower * powerFactor);
    }

    private void updateServos() {
        updateToggle(ToggleServo.State.CLOSED, ToggleServo.State.REST, pivotButton, batMobile.redSideArm.pivot);
        updateToggle(ToggleServo.State.CLOSED, ToggleServo.State.REST, pivotButton, batMobile.blueSideArm.pivot);
        updateToggle(ToggleServo.State.OPEN, ToggleServo.State.REST, clawButton, batMobile.redSideArm.claw);
        updateToggle(ToggleServo.State.OPEN, ToggleServo.State.REST, clawButton, batMobile.blueSideArm.claw);
        updateToggle(ToggleServo.State.CLOSED, ToggleServo.State.OPEN, depositButton, batMobile.depositServo);
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
        if (!imuInitialized()) {
            return;
        }

        if (setAngleSetpointButton.is(DOWN)) {
            degreesSetpoint = robot.imu.getHeading().getDegrees();
        }

        if (goToAngleButton.is(DOWN)) {
            double fullRotationOffset = GeneralMath.roundNearestMultiple(robot.imu.getIntegratedHeading().getDegrees(), 360);
            goingToAngle = true;
            anglePID = new PIDController(anglePIDCoefficients,  fullRotationOffset + degreesSetpoint);
            getAnglePIDPower();
        }

        if (!goToAngleButton.is(HELD) && goingToAngle && Math.abs(anglePID.getError()) < snapAngleError) {
            AutoRunner.log("PID Error", anglePID.getError());
            goingToAngle = false;
        }

        if ((liftIsUp && drivePowerFactor != lbDrivePower) || (liftWasUp && !liftIsUp) || (isInputting(gamepad1.right_stick_x))) {
            goingToAngle = false;
        }

        invertXY = goToAngleButton.is(HELD);
    }

    private double getAnglePIDPower() {
        return anglePID.getCorrectedOutput(robot.imu.getIntegratedHeading().getDegrees());
    }

    @Override
    protected void updateDrive() {
        if (toggleSideButton.is(DOWN)) {
            side = (side == AutoRunner.Side.RED ? AutoRunner.Side.BLUE : AutoRunner.Side.RED);
        }

        // up on the gamepad stick is negative
        drivePose.x = Math.pow(gamepad1.left_stick_x, 3) * drivePowerFactor;
        drivePose.y = -Math.pow(gamepad1.left_stick_y, 3) * drivePowerFactor;
        drivePose.r = Math.pow(gamepad1.right_stick_x, 3) * drivePowerFactor;
        if (invertXY) {
            double tempX = drivePose.x;
            drivePose.x = drivePose.y * (side == AutoRunner.Side.RED ? 1 : -1);
            drivePose.y = tempX * (side == AutoRunner.Side.RED ? -1 : 1);
        }
        if (imuInitialized() && !isInputting(drivePose.r) && goingToAngle) {
            drivePose.r = GeneralMath.round(getAnglePIDPower(), 2);
        }
        robot.driveTrain.drive(drivePose);
    }

    protected boolean imuInitialized() {
        return robot.imu != null;
    }

    private void updateTelemetry() {
        telemetry.addData("===P1","===");
        telemetry.addData("Drive Power", drivePowerFactor);
        telemetry.addData("GoingToAngle?", goingToAngle);
        telemetry.addData("Deposit up when intake?", toggleDepositUpWhenIntake);
        telemetry.addData("Angle PID Setpoint", anglePID != null ? degreesSetpoint : "?");
        telemetry.addData("IMU", initIMUAction.runIsComplete() ? robot.imu.getHeading().getDegrees() : "Initializing...");
        telemetry.addData("DriveR", drivePose.r);
        telemetry.addData("===P2","===");
        telemetry.addData("Lift State", state.name());
        telemetry.addData("Lift Power", getLiftInputPower());
        telemetry.addData("Lift Level", liftLevel);
        telemetry.addData("===DEBUG","===");
//        telemetry.addData("LeftClicks ", batMobile.elevator.left.getCurrentPosition());
//        telemetry.addData("RightClicks", batMobile.elevator.right.getCurrentPosition());
        telemetry.addData("Side", side.name());
        telemetry.addData("InvertXY?", invertXY);
        telemetry.addData("Battery", robot.getBatteryVoltage());
        telemetry.update();
    }


    class InitIMUOnThread extends Action {
        @Override
        protected void onRun() {
            robot.initializeIMU();
        }

        @Override
        protected boolean runIsComplete() {
            return robot.imuIsInitialized();
        }

        @Override
        protected void insideRun() throws SomethingBadHappened { }

        @Override
        protected void onEndRun() throws SomethingBadHappened { }
    }


}
