package org.firstinspires.ftc.teamcode.robot.BatMobile;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.hardware.servo.ToggleServo;

public class SideArm {

    public ToggleServo claw;
    public ToggleServo pivot;
    public TouchSensor pivotFeedback;

    public SideArm(HardwareMap hardwareMap) {
        pivotFeedback = hardwareMap.touchSensor.get("pivot feedback");
        claw = new ToggleServo(hardwareMap, "claw");
        pivot = new ToggleServo(hardwareMap, "pivot");
    }

    public boolean pivotIsDown() {
        return pivotFeedback.isPressed();
    }

}
