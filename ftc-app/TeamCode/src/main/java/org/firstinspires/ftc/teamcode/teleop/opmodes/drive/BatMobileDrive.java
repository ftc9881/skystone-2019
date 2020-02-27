package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

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
    private int outtakeMs;

    private Button pivotButton = new Button();
    private Button clawButton = new Button();
    private Button foundationButton = new Button();
    private Button capstoneButton = new Button();
    private Button depositButton = new Button();
    private Button toggleAnglePIDButton = new Button();
    private Button toggleToAngleWhenLiftUpButton = new Button();
    private Button toggleKeepAngleWhenDriving = new Button();
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

    private long timeAtLastIntake = 0;

    private InitIMUOnThread initIMUAction;
    private PIDController anglePID;
    private PIDCoefficients anglePIDCoefficients;
    private double degreesSetpoint = 0;
    private boolean goToAngleWhenLiftUp = true;
    private boolean keepAngleWhenDriving = true;
    private boolean goingToAngle = false;


    @Override
    protected void initialize() {
        super.initialize();
        batMobile = BatMobile.createInstance();

        deadZone = config.getDouble("dead zone", 0.1);

        lbDrivePower = config.getDouble("lb drive power", 0.5);
        rbDrivePower = config.getDouble("rb drive power", 0.25);
        liftIsUpDrivePower = config.getDouble("lift up drive power", Math.min(lbDrivePower, rbDrivePower));

        double m = config.getDouble("m battery drive power", 0.05);
        double b = config.getDouble("b battery drive power", 1.4);
        double batteryBasedDrivePower = -m * robot.getBatteryVoltage() + b;
        defaultDrivePower = config.getDouble("default drive power", batteryBasedDrivePower);

        slowLiftPowerFactor = config.getDouble("slow lift power", 0.3);
        slowLiftPowerZone = config.getDouble("slow lift zone", 0.7);
        outtakePowerFactor = config.getDouble("outtake power", 1.0);
        outtakeMs = config.getInt("outtake time", 500);
        anglePIDCoefficients = config.getKPID("angle");

        clicksPerLevel = config.getInt("clicks per level", 300);

        maxStickWindow = config.getDouble("max stick window", 0.97);
        fullDownTimeWindow = config.getDouble("full down time window", 300);

        batMobile.redSideArm.setPivotToInsideRestingPosition();
        batMobile.blueSideArm.setPivotToInsideRestingPosition();

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
        toggleAnglePIDButton.update(gamepad1.dpad_up);
        toggleToAngleWhenLiftUpButton.update(gamepad1.dpad_down);
        toggleKeepAngleWhenDriving.update(gamepad1.dpad_right);
        setAngleSetpointButton.update(gamepad1.dpad_left);
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
        double powerFactor = isInputting(gamepad2.right_trigger) ? slowLiftPowerFactor : 1.0;
        batMobile.elevator.setPowerLE(getLiftInputPower(), getExtendInputPower(), powerFactor);
    }

    private boolean isInputting(double input) {
        return Math.abs(input) > deadZone;
    }

    private void updateIntake() {
        double intakePower = (gamepad1.right_trigger - gamepad1.left_trigger);
        if (intakePower > 0) {
            batMobile.depositServo.set(ToggleServo.State.OPEN);
            timeAtLastIntake = System.currentTimeMillis();
        }
        if (!isInputting(intakePower) && System.currentTimeMillis() - timeAtLastIntake < outtakeMs) {
            batMobile.intake.setPower(-outtakePowerFactor);
        } else {
            batMobile.intake.setPower(intakePower * (intakePower < 0 ? outtakePowerFactor : 1));
        }
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

        if (toggleAnglePIDButton.is(DOWN)) {
            goToAngleWhenLiftUp = !goToAngleWhenLiftUp;
        }

        if (toggleKeepAngleWhenDriving.is(DOWN)) {
            keepAngleWhenDriving = !keepAngleWhenDriving;
        }

        if (setAngleSetpointButton.is(DOWN)) {
            degreesSetpoint = robot.imu.getHeading().getDegrees();
        }

        if (toggleToAngleWhenLiftUpButton.is(DOWN) || (liftIsUp && goToAngleWhenLiftUp)) {
            double fullRotationOffset = GeneralMath.roundNearestMultiple(robot.imu.getIntegratedHeading().getDegrees(), 360);
            anglePID = new PIDController(anglePIDCoefficients,  fullRotationOffset + degreesSetpoint);
            goingToAngle = true;
        }

        if (keepAngleWhenDriving && !goingToAngle) {
            anglePID = new PIDController(anglePIDCoefficients,  robot.imu.getIntegratedHeading().getDegrees());
            goingToAngle = true;
        }

        if ((liftIsUp && drivePowerFactor != lbDrivePower) || (liftWasUp && !liftIsUp) || (isInputting(gamepad1.right_stick_x))) {
            goingToAngle = false;
        }

    }

    private double getAnglePIDPower() {
        return anglePID.getCorrectedOutput(robot.imu.getIntegratedHeading().getDegrees());
    }

    @Override
    protected void updateDrive() {
        // up on the gamepad stick is negative
        drivePose.x = Math.pow(gamepad1.left_stick_x, 3) * drivePowerFactor;
        drivePose.y = -Math.pow(gamepad1.left_stick_y, 3) * drivePowerFactor;
        drivePose.r = Math.pow(gamepad1.right_stick_x, 3) * drivePowerFactor;
        if (imuInitialized() && !isInputting(drivePose.r) && goingToAngle) {
            drivePose.r = GeneralMath.round(getAnglePIDPower(), 2);
        }
        robot.driveTrain.drive(drivePose);
    }

    private boolean imuInitialized() {
        return robot.imu != null;
    }


    private void updateTelemetry() {
        telemetry.addData("===P1","===");
        telemetry.addData("Drive Power", drivePowerFactor);
        telemetry.addData("Go To Angle when Lift Raised?", goToAngleWhenLiftUp);
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
