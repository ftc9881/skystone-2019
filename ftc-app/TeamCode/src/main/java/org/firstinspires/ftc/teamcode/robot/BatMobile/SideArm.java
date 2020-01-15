package org.firstinspires.ftc.teamcode.robot.BatMobile;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.devices.ToggleServo;
import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;

public class SideArm {

    public ToggleServo claw;
    public ToggleServo pivot;
    private double pivotFoundationPosition;

    public SideArm(HardwareMap hardwareMap) {
        claw = new ToggleServo(hardwareMap, "claw");
        pivot = new ToggleServo(hardwareMap, "pivot");
        Configuration config = new Configuration("HardwareConstants");
        pivotFoundationPosition = config.getDouble("pivot foundation", 0.0);
    }

}
