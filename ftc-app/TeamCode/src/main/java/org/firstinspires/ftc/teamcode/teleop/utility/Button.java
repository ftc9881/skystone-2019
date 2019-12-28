package org.firstinspires.ftc.teamcode.teleop.utility;

public class Button {

    public enum State // button states
    {
        DOWN,   // moment press down
        HELD,   // continued press down
        UP,     // moment of release
        OFF,    // continued release
        NOT_INITIALIZED
    }

    private State state;

    public Button() {
        state = State.NOT_INITIALIZED;
    }

    public State update(boolean buttonPressed) {
        if (buttonPressed) {
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

}
