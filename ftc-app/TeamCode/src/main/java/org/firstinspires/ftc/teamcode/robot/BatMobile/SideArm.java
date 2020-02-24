package org.firstinspires.ftc.teamcode.robot.BatMobile;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.hardware.servo.ToggleServo;
import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;

public class SideArm {

    public ToggleServo claw;
    public ToggleServo pivot;
    public TouchSensor pivotFeedback;
    private double pivotInsideRestingPosition;
    private double pivotDeployIntakePosition;

    public SideArm(AutoRunner.Side side, HardwareMap hardwareMap) {
        pivotFeedback = hardwareMap.touchSensor.get(side.getKey() + " pivot feedback");
        claw = new ToggleServo(hardwareMap, side.getKey() + " claw");
        pivot = new ToggleServo(hardwareMap, side.getKey() + " pivot");

        Configuration hardwareConstantsConfig = new Configuration("HardwareConstants");
        pivotInsideRestingPosition = hardwareConstantsConfig.getDouble("pivot inside", 0.31);
        pivotDeployIntakePosition = hardwareConstantsConfig.getDouble("pivot deploy", 0.31);
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
