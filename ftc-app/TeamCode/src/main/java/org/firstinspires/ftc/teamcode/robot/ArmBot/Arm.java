package org.firstinspires.ftc.teamcode.robot.ArmBot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    public DcMotor pivotMotor;

    public Arm(HardwareMap hardwareMap) {
        pivotMotor = hardwareMap.dcMotor.get("pivot");
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setDirection(DcMotor.Direction.FORWARD);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setPower(0);
    }

}
