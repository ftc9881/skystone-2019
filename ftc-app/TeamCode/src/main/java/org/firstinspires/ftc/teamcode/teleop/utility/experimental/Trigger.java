package org.firstinspires.ftc.teamcode.teleop.utility.experimental;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Value of a trigger is a positive float from 0 to 1.
 */
public class Trigger implements IInput {

    public enum Input {
        LEFT_TRIGGER, RIGHT_TRIGGER
    }

    private Gamepad gamepad;
    private Input input;

    public Trigger(Gamepad gamepad, Input input) {
        this.gamepad = gamepad;
        this.input = input;
    }

    @Override
    public double getValue() {
        switch (input) {
            case LEFT_TRIGGER:
                return gamepad.left_trigger;
            case RIGHT_TRIGGER:
                return gamepad.right_trigger;
            default:
                return 0.0;
        }
    }
}
