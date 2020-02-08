package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

import static org.firstinspires.ftc.teamcode.teleop.utility.Button.State.DOWN;

@TeleOp(group="Test")
public class LiftControlTest extends TeleOpBase {

    private BatMobile batMobile;

    private double slowLiftPowerFactor;
    private int curve;

    private Button incrementCurve = new Button();
    private Button decrementCurve = new Button();

    @Override
    protected void initialize() {
        batMobile = BatMobile.getInstance();
        slowLiftPowerFactor = config.getDouble("slow lift power", 0.3);
        updateButtons();
    }

    @Override
    protected void update() {
        updateButtons();
        updateCurveLevel();
        updateElevator();
        updateTelemetry();
    }

    private void updateButtons() {
        incrementCurve.update(gamepad2.dpad_up);
        decrementCurve.update(gamepad2.dpad_down);
    }

    private void updateElevator() {
        updateElevatorManual();
    }

    private void updateCurveLevel() {
        if (incrementCurve.is(DOWN)) {
            curve += 1;
        }
        if (decrementCurve.is(DOWN)) {
            curve -= 1;
        }
    }

    private void updateElevatorManual() {
        double liftPowerP2 = -gamepad2.left_stick_y;
        double extendPowerP2 = gamepad2.right_stick_x;
        double powerFactor = isInputting(gamepad2.right_trigger) || isInputting(gamepad2.left_trigger) ? slowLiftPowerFactor : 1.0;
        batMobile.elevator.setPowerLE(Math.pow(liftPowerP2, curve), Math.pow(extendPowerP2, curve), powerFactor);
    }

    private boolean isInputting(double input) {
        return Math.abs(input) > 0.1;
    }

    private void updateTelemetry() {
        telemetry.addData("Curve", curve);
        telemetry.addData("Left Clicks", batMobile.elevator.left.getCurrentPosition());
        telemetry.addData("Right Clicks", batMobile.elevator.right.getCurrentPosition());
        telemetry.update();
    }

}
