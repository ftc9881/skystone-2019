package org.firstinspires.ftc.teamcode.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;

public class OdometryWheel {

    private DcMotor motor;
    private double clicksToInches;

    public OdometryWheel(DcMotor motor) {
        this.motor = motor;
        motor.setDirection(DcMotor.Direction.FORWARD);
        resetEncoder();
        Configuration config = new Configuration("HardwareConstants");
        clicksToInches = config.getDouble("odometry clicks per inch", 250);
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }

    public double getInches() {
        return getPosition() / clicksToInches;
    }

    public DcMotor.Direction getDirection() {
        return motor.getDirection();
    }

    public DcMotor getMotor() {
        return motor;
    }

    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
