package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FoundationGrabber {

    private static final double GRAB_POSITION = 0;
    private static final double RELEASE_POSITION = 0;

    public Servo miniGrabServo;
    public Servo mainGrabServo;

    public FoundationGrabber (HardwareMap hardwareMap) {
        miniGrabServo = hardwareMap.servo.get("mini grab");
        mainGrabServo = hardwareMap.servo.get("main grab");
    }

    public void grab() {
        mainGrabServo.setPosition(GRAB_POSITION);
    }

    public void release() {
        mainGrabServo.setPosition(RELEASE_POSITION);
    }

}
