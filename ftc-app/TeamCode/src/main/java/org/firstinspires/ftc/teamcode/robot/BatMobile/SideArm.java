package org.firstinspires.ftc.teamcode.robot.BatMobile;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.servo.ToggleServo;
import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;

public class SideArm {

    public ToggleServo claw;
    public ToggleServo pivot;

    public SideArm(HardwareMap hardwareMap) {
        claw = new ToggleServo(hardwareMap, "claw");
        pivot = new ToggleServo(hardwareMap, "pivot");
        Configuration config = new Configuration("HardwareConstants");
    }

}
