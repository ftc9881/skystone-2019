package org.firstinspires.ftc.teamcode.math;

import org.firstinspires.ftc.teamcode.teleop.utility.Command;

public class PIDController {
    private double currentTime, previousTime, deltaTime;
    private double integral = 0;
    private double currentError, previousError, deltaError, derivative;
    private double kP, kI, kD, targetValue;

    public PIDController (double kP, double kI, double kD, double targetValue) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
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
        deltaError = currentError - previousError;
        derivative =  deltaError/deltaTime;

        integral += currentError * deltaTime;

        previousError = currentError;

        double output = -(currentError * kP + integral * kI + derivative * kD);
        return output;
    }

    private double getTimeSeconds() {
        return System.currentTimeMillis() / 1000.0;
    }

}
