package org.firstinspires.ftc.teamcode.math;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.teleop.utility.Command;

public class PIDController {
    private double currentTime, previousTime, deltaTime;
    private double integral = 0;
    private double currentError, previousError, deltaError, derivative;
    private double kP, kI, kD, targetValue;
    private double previousOutput;

    public PIDController (PIDCoefficients kPID, double targetValue) {
        this.kP = kPID.p;
        this.kI = kPID.i;
        this.kD = kPID.d;
        this.targetValue = targetValue;
        this.currentTime = getTimeSeconds();
    }


    public PIDController (Command config, double targetValue) {
        this.kP = config.getDouble("kp", 0);
        this.kI = config.getDouble("ki", 0);
        this.kD = config.getDouble("kd", 0);
        this.targetValue = targetValue;
        this.currentTime = getTimeSeconds();
    }

    public PIDController (Command config, String name, double targetValue) {
        this.kP = config.getDouble(name + " kp", 0);
        this.kI = config.getDouble(name + " ki", 0);
        this.kD = config.getDouble(name + " kd", 0);
        this.targetValue = targetValue;
        this.currentTime = getTimeSeconds();
    }

    public double getCorrectedOutput(double processValue) {
        previousTime = currentTime;
        currentTime = getTimeSeconds();
        deltaTime = currentTime - previousTime;
        currentError = processValue - targetValue;

        if (deltaTime > 0) {
            deltaError = currentError - previousError;

            derivative =  deltaError/deltaTime;
            integral += currentError * deltaTime;

            double output = -(currentError * kP + integral * kI + derivative * kD);

            previousError = currentError;
            previousOutput = output;

            return output;
        }

        return previousOutput;
    }

    public void reset() {
        currentTime = getTimeSeconds();
        previousError = currentTime;
        deltaTime = 0;
        previousError = currentError;
        derivative = 0;
        integral = 0;
    }

    public double getError() {
        return currentError;
    }

    public double getTargetValue() {
        return targetValue;
    }

    private double getTimeSeconds() {
        return System.currentTimeMillis() / 1000.0;
    }

}
