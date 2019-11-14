package org.firstinspires.ftc.teamcode.math;

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
