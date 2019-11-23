package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FoundationGrabber {

    public enum State {
        GRABBING(0.0),
        RELEASED(0.4),
        FULLY_IN(0.7);

        final double position;

        State(double position) {
            this.position = position;
        }
    }

    public Servo leftServo;
    public Servo rightServo;

    private State currentState;

    public FoundationGrabber (HardwareMap hardwareMap) {
        leftServo = hardwareMap.servo.get("left grab");
        rightServo = hardwareMap.servo.get("right grab");

//        leftServo.setPosition(State.FULLY_IN.position);
//        rightServo.setPosition(State.FULLY_IN.position);
    }

    public void set(State state) {
        leftServo.setPosition(state.position);
        rightServo.setPosition(state.position);
        currentState = state;
    }

    public void toggle() {
        State newState = currentState == State.GRABBING ? State.RELEASED : State.GRABBING;
        set(newState);
    }

}
