package org.firstinspires.ftc.teamcode.teleop.utility;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Value of axis is a float from -1 to 1.
 */
public class Axis implements IInput {

    public enum Input {
        LEFT_STICK_X, RIGHT_STICK_X,
        LEFT_STICK_Y, RIGHT_STICK_Y
    }

    private Gamepad gamepad;
    private IInput negativeInput;
    private IInput positiveInput;
    private Input axisInput;

    public Axis(Gamepad gamepad, Input axisInput) {
        this.gamepad = gamepad;
        this.axisInput = axisInput;
    }

    public Axis(IInput negativeInput, IInput positiveInput) {
        this.negativeInput = negativeInput;
        this.positiveInput = positiveInput;
    }

    @Override
    public double getValue() {
        if (negativeInput != null && positiveInput != null) {
            return negativeInput.getValue() - positiveInput.getValue();
        }
        if (gamepad != null) {
            switch (axisInput) {
                case LEFT_STICK_X:
                    return gamepad.left_stick_x;
                case LEFT_STICK_Y:
                    return gamepad.left_stick_y;
                case RIGHT_STICK_X:
                    return gamepad.right_stick_x;
                case RIGHT_STICK_Y:
                    return gamepad.right_stick_y;
            }
        }
        return 0.0;
    }

}

