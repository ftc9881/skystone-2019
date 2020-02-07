package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.teleop.opmodes.drive.BaseDrive;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

import static org.firstinspires.ftc.teamcode.teleop.utility.Button.State.DOWN;

@TeleOp(group="Test")
public class ElevatorTest extends TeleOpBase {

    private BatMobile batMobile;

    private double slowLiftPowerFactor;
    private double deadZone;
    private int clicksPerLiftLevel;
    private int liftLevel = 0;
    private boolean isElevatorAutoMode = false;
    private boolean isLifted = false;
    private boolean lastLiftInputWasUp = false ;

    private Button increaseLiftLevelButton = new Button();
    private Button decreaseLiftLevelButton = new Button();
    private Button toggleLiftButton = new Button();

    @Override
    protected void initialize() {
        batMobile = BatMobile.getInstance();
        clicksPerLiftLevel = config.getInt("clicks per lift level", 0);
        slowLiftPowerFactor = config.getDouble("slow lift power", 0.3);
        deadZone = config.getDouble("dead zone", 0.1);
        updateButtons();
    }

    @Override
    protected void update() {
        updateButtons();
        updateElevator();
        updateTelemetry();
    }

    private void updateButtons() {
        increaseLiftLevelButton.update(gamepad2.dpad_up);
        decreaseLiftLevelButton.update(gamepad2.dpad_down);
        toggleLiftButton.update(gamepad2.a);
    }

    private void updateElevator() {
        updateElevatorLevels();

        if (isManuallyInputtingForElevator()) {
            isElevatorAutoMode = false;
        } else if (lastLiftInputWasUp) {
            holdLiftPosition();
        }

        if (isElevatorAutoMode) {
            updateElevatorShortcuts();
        } else {
            updateElevatorManual();
        }
    }

    private void updateElevatorLevels() {
        if (increaseLiftLevelButton.is(DOWN)) {
            liftLevel += 1;
        }
        if (decreaseLiftLevelButton.is(DOWN)) {
            liftLevel -= 1;
        }
    }

    private boolean isManuallyInputtingForElevator() {
        return gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left ||
                Math.abs(gamepad2.left_stick_y) > deadZone || Math.abs(gamepad2.right_stick_x) > deadZone;
    }

    private void holdLiftPosition() {
        batMobile.elevator.runToRelativePosition(0, 0);
    }

    private void updateElevatorManual() {
        double liftPowerP2 = -gamepad2.left_stick_y;
        double extendPowerP2 = gamepad2.right_stick_x;
        double powerFactor = isInputting(gamepad2.right_trigger) || isInputting(gamepad2.left_trigger) ? slowLiftPowerFactor : 1.0;
        batMobile.elevator.setPowerLE(liftPowerP2, extendPowerP2, powerFactor);
        lastLiftInputWasUp = liftPowerP2 > 0;
    }

    private boolean isInputting(double input) {
        return Math.abs(input) > deadZone;
    }

    private void updateElevatorShortcuts() {
        if (toggleLiftButton.is(DOWN)) {
            isLifted = !isLifted;
            if (isLifted) {
                liftLevel += 1;
            }
            int level = isLifted ? liftLevel : -liftLevel;
            batMobile.elevator.runToRelativePosition(level * clicksPerLiftLevel, 0);
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Left Clicks", batMobile.elevator.left.getCurrentPosition());
        telemetry.addData("Right Clicks", batMobile.elevator.right.getCurrentPosition());
        telemetry.update();
    }

}
