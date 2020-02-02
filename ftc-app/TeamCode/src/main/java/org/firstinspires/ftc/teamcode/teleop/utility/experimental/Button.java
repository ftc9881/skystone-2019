package org.firstinspires.ftc.teamcode.teleop.utility.experimental;

import com.qualcomm.robotcore.hardware.Gamepad;


/**
 * The Button provides a way to get the moment of press or release.
 * However, the button's state must be updated every frame.
 *
 * @author Trinity Chung
 * @version 1.0
 */
public class Button implements IInput {

    public enum Input {
        A, B, X, Y,
        LEFT_STICK, RIGHT_STICK,
        LEFT_BUMPER, RIGHT_BUMPER,
        DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,
    }

    public enum State // button states
    {
        DOWN,   // moment press down
        HELD,   // continued press down
        UP,     // moment of release
        OFF,    // continued release
        NOT_INITIALIZED
    }

    private State state;
    private Gamepad gamepad;
    private Input input;

    public Button(Gamepad gamepad, Input input) {
        this.gamepad = gamepad;
        this.input = input;
        this.state = State.NOT_INITIALIZED;
    }

    State update() {
        if (isPressed()) {
            if (state == State.OFF || state == State.UP || state == State.NOT_INITIALIZED)
                state = state.DOWN;
            else
                state = state.HELD;
        }
        else {
            if (state == State.HELD || state == State.DOWN)
                state = State.UP;
            else
                state = State.OFF;
        }
        return state;
    }

    public boolean is(State state) {
        return this.state == state;
    }

    @Override
    public double getValue() {
        switch (state) {
            case DOWN:
            case HELD:
                return 1.0;
            case UP:
            case OFF:
            case NOT_INITIALIZED:
            default:
                return 0.0;
        }
    }

    public boolean isPressed() {
        switch (input) {
            case A:
                return gamepad.a;
            case B:
                return gamepad.b;
            case X:
                return gamepad.x;
            case Y:
                return gamepad.y;
            case LEFT_STICK:
                return gamepad.left_stick_button;
            case RIGHT_STICK:
                return gamepad.right_stick_button;
            case LEFT_BUMPER:
                return gamepad.left_bumper;
            case RIGHT_BUMPER:
                return gamepad.right_bumper;
            case DPAD_UP:
                return gamepad.dpad_up;
            case DPAD_DOWN:
                return gamepad.dpad_down;
            case DPAD_LEFT:
                return gamepad.dpad_left;
            case DPAD_RIGHT:
                return gamepad.dpad_right;
            default:
                return false;
        }
    }

}
