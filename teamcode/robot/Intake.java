package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake implements IRobotSystem {

    public DcMotor left;
    public DcMotor right;

    public Intake(HardwareMap hardwareMap) {
        left = hardwareMap.dcMotor.get("left intake");
        right = hardwareMap.dcMotor.get("right intake");
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
