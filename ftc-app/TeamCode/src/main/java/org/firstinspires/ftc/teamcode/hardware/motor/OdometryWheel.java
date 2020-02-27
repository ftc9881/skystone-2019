package org.firstinspires.ftc.teamcode.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;

public class OdometryWheel {

    private CachingMotorEx motor;
    private double clicksToInches;

    public OdometryWheel(CachingMotorEx motor) {
        this.motor = motor;
        motor.setDirection(DcMotor.Direction.FORWARD);
        resetEncoder();
        Configuration config = new Configuration("HardwareConstants");
        clicksToInches = config.getDouble("odometry clicks per inch", 250);
    }

    public double getClicksToInches() {
        return clicksToInches;
    }

    public int getClicks() {
        return motor.getCurrentPosition();
    }

    public double getInches() {
        return getClicks() / clicksToInches;
    }

    public DcMotor.Direction getDirection() {
        return motor.getDirection();
    }

    public CachingMotorEx getMotor() {
        return motor;
    }

    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
