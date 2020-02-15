package org.firstinspires.ftc.teamcode.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;

public class OdometryWheel {

    private DcMotor motor;

    public OdometryWheel(DcMotor motor) {
        this.motor = motor;
        resetEncoder();
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }

    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
