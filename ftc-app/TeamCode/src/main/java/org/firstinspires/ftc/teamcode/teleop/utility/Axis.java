package org.firstinspires.ftc.teamcode.teleop.utility;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;

/**
 * Value of axis is a float from -1 to 1.
 */
public class Axis implements IInput {

    public enum Input {
        LEFT_STICK_X, RIGHT_STICK_X,
        LEFT_STICK_Y, RIGHT_STICK_Y,
        CUSTOM
    }

    private Gamepad gamepad;
    private IInput negativeInput;
    private IInput positiveInput;
    private Input axisInput;

    public Axis(Gamepad gamepad, Input axisInput) {
        this.gamepad = gamepad;
        this.axisInput = axisInput;
        AutoRunner.log("axisInput", axisInput);
    }

    public Axis(IInput negativeInput, IInput positiveInput) {
        this.negativeInput = negativeInput;
        this.positiveInput = positiveInput;
        this.axisInput = Input.CUSTOM;
        AutoRunner.log("axisInput", axisInput);
    }

    @Override
    public double getValue() {
        switch (axisInput) {
            case LEFT_STICK_X:
                return gamepad.left_stick_x;
            case LEFT_STICK_Y:
                return gamepad.left_stick_y;
            case RIGHT_STICK_X:
                return gamepad.right_stick_x;
            case RIGHT_STICK_Y:
                return gamepad.right_stick_y;
            case CUSTOM:
                return positiveInput.getValue() - negativeInput.getValue();
            default:
                throw new IllegalArgumentException("Axis input is not implemented/defined");
        }
    }

}

