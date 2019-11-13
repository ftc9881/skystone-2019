package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FoundationGrabber implements IRobotSystem {

    private static final double GRAB_POSITION = 0;
    private static final double RELEASE_POSITION = 0;

    public Servo grabServo;

    public FoundationGrabber (HardwareMap hardwareMap) {
        grabServo = hardwareMap.servo.get("grabber");
    }

    public void grab() {
        grabServo.setPosition(GRAB_POSITION);
    }

    public void release() {
        grabServo.setPosition(RELEASE_POSITION);
    }

}
