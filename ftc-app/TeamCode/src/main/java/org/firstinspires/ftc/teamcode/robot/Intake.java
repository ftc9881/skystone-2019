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
        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power) {
        left.setPower(power);
        right.setPower(power);
    }

    public void stop() {
        setPower(0);
    }

}
