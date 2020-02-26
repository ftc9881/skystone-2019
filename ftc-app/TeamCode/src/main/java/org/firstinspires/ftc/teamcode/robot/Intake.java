package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.hardware.motor.CachingMotor;
import org.firstinspires.ftc.teamcode.hardware.motor.CachingMotorEx;

public class Intake {

    public CachingMotorEx left;
    public CachingMotorEx right;

    public Intake(HardwareMap hardwareMap) {
        AutoRunner.log("Create Intake");

        left = new CachingMotorEx(hardwareMap, "left intake");
        right = new CachingMotorEx(hardwareMap, "right intake");
        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.FORWARD);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power) {
        left.setPower(power);
        right.setPower(-power);
    }

    public void stop() {
        setPower(0);
    }

}
