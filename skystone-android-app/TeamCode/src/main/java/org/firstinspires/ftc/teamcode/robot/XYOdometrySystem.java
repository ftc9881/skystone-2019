package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class XYOdometrySystem {

    private DcMotor xEncoder;
    private DcMotor yEncoder;

    public XYOdometrySystem(HardwareMap hardwareMap) {
        // The encoders are plugged into where the intake motors are
        xEncoder = hardwareMap.dcMotor.get("left intake");
        yEncoder = hardwareMap.dcMotor.get("right intake");
        resetEncoders();
    }

    public void resetEncoders() {
        xEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        xEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getX() {
        return xEncoder.getCurrentPosition();
    }

    public double getY() {
        return yEncoder.getCurrentPosition();
    }
}
