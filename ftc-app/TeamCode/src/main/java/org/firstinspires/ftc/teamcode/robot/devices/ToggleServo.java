package org.firstinspires.ftc.teamcode.robot.devices;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;

public class ToggleServo {

    public enum State { OPEN, CLOSED }

    public Servo servo;
    private State currentState;
    private double openPosition;
    private double closedPosition;

    public ToggleServo(HardwareMap hardwareMap, String name) {
        servo = hardwareMap.servo.get(name);
        Configuration config = new Configuration("HardwareConstants");
        openPosition = config.getDouble(name + " open", 0.0);
        closedPosition = config.getDouble(name + " closed", 1.0);
    }

    public void set(State state) {
        double position = state == State.OPEN ? openPosition : closedPosition;
        servo.setPosition(position);
        currentState = state;
    }

    public void toggle() {
        State newState = currentState == State.OPEN ? State.CLOSED : State.OPEN;
        set(newState);
    }

}
