package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.motor.CachingMotor;

public class Intake {

    public CachingMotor left;
    public CachingMotor right;

    public Intake(HardwareMap hardwareMap) {
        left = new CachingMotor(hardwareMap, "left intake");
        right = new CachingMotor(hardwareMap, "right intake");
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
