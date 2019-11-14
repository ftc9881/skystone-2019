package org.firstinspires.ftc.teamcode.auto.actions;

import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;

public class RelativeTurn extends Action {

    private static final double ERROR_RANGE_RADIANS = 0.1;
    private static final double kP = 0.2;

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
    }

    @Override
    protected boolean runIsComplete() {
        AutoRunner.log("Current Angle", currentRadians);
        AutoRunner.log("Angle to turn", radiansToTurn);
        double errorRadians = Math.abs(currentRadians - radiansToTurn);
        return errorRadians < ERROR_RANGE_RADIANS;
    }

    @Override
    protected void insideRun() {
        currentRadians = robot.getImuHeading().getRadians();
        double error = (radiansToTurn + initialRadians) - currentRadians;
        double errorCorrectedPower = (error * kP) + kP * (error > 0 ? 1 : -1);
        Pose drivePose = new Pose(0, 0, errorCorrectedPower);
        robot.driveTrain.drive(drivePose, powerFactor);
    }

    @Override
    protected void onEndRun() {
        robot.driveTrain.stop();
    }
}
