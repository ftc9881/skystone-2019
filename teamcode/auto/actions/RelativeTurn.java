package org.firstinspires.ftc.teamcode.auto.actions;

import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;

public class RelativeTurn extends Action {

    private static final double ERROR_RANGE_RADIANS = 0.1;
    private static final double kP = 0.1;

    Robot robot;
    private double radiansToTurn;
    private double initialRadians;
    private double currentRadians;
    private double powerFactor;


    public RelativeTurn(Angle angleToTurn, double powerFactor) {
        this.robot = Robot.getInstance();
        this.radiansToTurn = angleToTurn.getRadians();
        this.powerFactor = powerFactor;
    }

    @Override
    protected void onRun() {
        initialRadians = robot.getImuHeading().getRadians();
        currentRadians = robot.getImuHeading().getRadians();
        AutoRunner.log("AngleToTurn", radiansToTurn);
        AutoRunner.log("CurrentAngle", currentRadians);
    }

    @Override
    protected boolean runIsComplete() {
        double errorRadians = Math.abs(currentRadians - radiansToTurn);
        return errorRadians < ERROR_RANGE_RADIANS;
    }

    @Override
    protected void insideRun() {
        currentRadians = robot.getImuHeading().getRadians();
        double error = currentRadians - (radiansToTurn + initialRadians);
        double errorCorrectedPower = (error * kP) + kP * (error > 0 ? 1 : -1);
        Pose drivePose = new Pose(0, 0, errorCorrectedPower);
        robot.driveTrain.drive(drivePose, powerFactor);
        AutoRunner.log("TurnError", error);
        AutoRunner.log("TurnPower", errorCorrectedPower);
    }

    @Override
    protected void onEndRun() {
        robot.driveTrain.stop();
    }
}
