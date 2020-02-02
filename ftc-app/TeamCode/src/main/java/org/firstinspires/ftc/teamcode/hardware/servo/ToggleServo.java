package org.firstinspires.ftc.teamcode.hardware.servo;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;

public class ToggleServo {

    public enum State { OPEN, CLOSED, REST, NONE }

    public CachingServo servo;

    private State currentState = State.NONE;
    private double openPosition;
    private double closedPosition;
    private double restPosition;

    public ToggleServo(HardwareMap hardwareMap, String name) {
        servo = new CachingServo(hardwareMap, name);
        Configuration config = new Configuration("HardwareConstants");
        openPosition = config.getDouble(name + " open", 0.0);
        closedPosition = config.getDouble(name + " closed", 1.0);
        restPosition = config.getDouble(name + " rest", closedPosition);
    }

    public State getState() {
        return currentState;
    }

    public void set(State state) {
        currentState = state;
        switch (state) {
            case OPEN:
                servo.setPosition(openPosition);
                break;
            case CLOSED:
                servo.setPosition(closedPosition);
                break;
            case REST:
                servo.setPosition(restPosition);
                break;
        }
    }

    public void toggle() {
        toggle(State.CLOSED, State.OPEN);
    }

    public void toggle(State a, State b) {
        State newState = currentState == a ? b : a;
        set(newState);
    }

    public static State stringToState(String string) {
        string = string.toUpperCase();
        State state = State.REST;
        switch (string) {
            case "OPEN":
                state = State.OPEN;
                break;
            case "CLOSED":
                state = State.CLOSED;
                break;
        }
        return state;
    }

}
