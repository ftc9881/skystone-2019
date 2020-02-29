package org.firstinspires.ftc.teamcode.math;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;

public class MoveController {
    private double underkP, overkP, targetValue;
    private boolean setStarting;
    private boolean isStartingPositive;

    public MoveController(Command config, String name, double targetValue) {
        double kP = config.getDouble(name + " kp", 0);
        this.underkP = config.getDouble(name + " kp under", kP);
        this.overkP = config.getDouble(name + " kp over", kP);
        this.targetValue = targetValue;
        setStarting = false;
    }

    public double getCorrectedOutput(double processValue) {
        if (!setStarting) {
            isStartingPositive = processValue - targetValue > 0;
            setStarting = true;
        }

        double error = processValue - targetValue;

        double kP = (error > 0 && isStartingPositive || error < 0 && !isStartingPositive) ? underkP : overkP;
        AutoRunner.log("MoveController", kP);

        return -error * kP;
    }

    public double getTargetValue() {
        return targetValue;
    }

}
