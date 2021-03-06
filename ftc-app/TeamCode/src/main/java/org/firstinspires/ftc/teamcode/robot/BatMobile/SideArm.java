package org.firstinspires.ftc.teamcode.robot.BatMobile;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.hardware.servo.ToggleServo;
import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;

public class SideArm {

    public ToggleServo claw;
    public ToggleServo pivot;
    private TouchSensor pivotFeedback;
    private double pivotInsideRestingPosition;
    private double pivotDeployIntakePosition;

    SideArm(AutoRunner.Side side, HardwareMap hardwareMap) {
        pivotFeedback = hardwareMap.touchSensor.get(side.getKey() + " pivot feedback");
        claw = new ToggleServo(hardwareMap, side.getKey() + " claw");
        pivot = new ToggleServo(hardwareMap, side.getKey() + " pivot");

        Configuration hardwareConstantsConfig = new Configuration("HardwareConstants");
        pivotInsideRestingPosition = hardwareConstantsConfig.getDouble(side.getKey() + " pivot inside", 0);
        pivotDeployIntakePosition = hardwareConstantsConfig.getDouble(side.getKey() + " pivot deploy", 0);
    }

    public boolean feedbackIsPressed() {
        return pivotFeedback.isPressed();
    }

    public void setPivotToInsideRestingPosition() {
        pivot.servo.setPosition(pivotInsideRestingPosition);
    }

    public void setPivotToDeployIntakePosition() {
        pivot.servo.setPosition(pivotDeployIntakePosition);
    }

}
