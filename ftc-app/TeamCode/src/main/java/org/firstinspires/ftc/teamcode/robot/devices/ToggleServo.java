package org.firstinspires.ftc.teamcode.robot.devices;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;

public class ToggleServo {

    public enum State { OPEN, CLOSED, REST }

    public Servo servo;

    private String name;
    private State currentState;
    private double openPosition;
    private double closedPosition;
    private double restPosition;

    public ToggleServo(HardwareMap hardwareMap, String name) {
        this.name = name;
        servo = hardwareMap.servo.get(name);
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
        double position = restPosition;
        switch (state) {
            case OPEN:
                position = openPosition;
                break;
            case CLOSED:
                position = closedPosition;
                break;
        }
        servo.setPosition(position);
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
