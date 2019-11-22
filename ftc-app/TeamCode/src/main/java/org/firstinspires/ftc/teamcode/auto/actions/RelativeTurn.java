package org.firstinspires.ftc.teamcode.auto.actions;

import org.firstinspires.ftc.teamcode.auto.structure.AutoOpConfiguration;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;

public class RelativeTurn extends Action {

    private static final double ERROR_RANGE_RADIANS = 0.1;

    Robot robot;
    private PIDController pidController;
    private double targetAngle;
    private double initialRadians;
    private double currentRadians;
    private double powerFactor;

    public RelativeTurn(Angle angleToTurn, double powerFactor) {
        this.robot = Robot.getInstance();
        this.targetAngle = angleToTurn.getRadians();
        this.powerFactor = powerFactor;

        AutoOpConfiguration config = AutoOpConfiguration.getInstance();
        double kP = config.properties.getDouble("turn kp", 0);
        double kI = config.properties.getDouble("turn ki", 0);
        double kD = config.properties.getDouble("turn kd", 0);
        pidController = new PIDController(kP ,kI ,kD, targetAngle);
    }

    @Override
    protected void onRun() {
        initialRadians = robot.getImuHeading().getRadians();
        currentRadians = robot.getImuHeading().getRadians();
        AutoRunner.log("AngleToTurn", targetAngle);
        AutoRunner.log("CurrentAngle", currentRadians);
    }

    @Override
    protected boolean runIsComplete() {
        double errorRadians = Math.abs(currentRadians - targetAngle);
        return errorRadians < ERROR_RANGE_RADIANS;
    }

    @Override
    protected void insideRun() {
        currentRadians = robot.getImuHeading().getRadians();
        double errorCorrectedPower = pidController.getCorrectedOutput(currentRadians);
        Pose drivePose = new Pose(0, 0, errorCorrectedPower);
        robot.driveTrain.drive(drivePose);
        AutoRunner.log("TurnPower", errorCorrectedPower);
    }

    @Override
    protected void onEndRun() {
        robot.driveTrain.stop();
    }
}
