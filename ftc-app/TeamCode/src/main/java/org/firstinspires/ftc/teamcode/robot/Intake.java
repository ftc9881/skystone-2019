package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    public DcMotor left;
    public DcMotor right;

    public Intake(HardwareMap hardwareMap) {
        left = hardwareMap.dcMotor.get("left intake");
        right = hardwareMap.dcMotor.get("right intake");

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        left.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void in() {
        left.setPower(1);
        right.setPower(1);
    }

    public void out() {
        left.setPower(-1);
        right.setPower(-1);
    }

    public void stop() {
        left.setPower(0);
        right.setPower(0);
    }

}
