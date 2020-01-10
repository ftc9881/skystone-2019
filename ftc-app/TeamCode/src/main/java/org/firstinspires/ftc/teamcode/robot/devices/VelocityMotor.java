package org.firstinspires.ftc.teamcode.robot.devices;

import com.qualcomm.robotcore.hardware.DcMotor;

public class VelocityMotor {

    public DcMotor motor;
    private double currentTime = 0;
    private double previousTime = 0;
    private int currentClicks = 0;
    private int previousClicks = 0;

    private double velocity;

    public VelocityMotor(DcMotor motor) {
        this.motor = motor;
        currentTime = System.currentTimeMillis();
    }

    public void update() {
        previousClicks = currentClicks;
        currentClicks = motor.getCurrentPosition();

        previousTime = currentTime;
        currentTime = System.currentTimeMillis();

        velocity = (currentClicks - previousClicks) / (currentTime - previousTime);
    }

    public double getVelocity() {
        return velocity;
    }

}
