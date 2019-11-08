package org.firstinspires.ftc.teamcode.auto.actions;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;

public class RelativeTurn extends Action {

    private static final double DEGREES_ERROR_RANGE = 5;
    private static final double kP = 0.2;

    Robot robot;
    private double angleToTurn;
    private double initialAngle;
    private double currentAngle;
    private double power;


    public RelativeTurn(Robot robot, double angleToTurn, double power) {
        this.robot = robot;
        this.angleToTurn = angleToTurn;
        this.power = power;
    }

    @Override
    protected void onRun() {
        initialAngle = robot.imu.getAngularOrientation().firstAngle;
        currentAngle = robot.imu.getAngularOrientation().firstAngle;
    }

    @Override
    protected boolean runIsComplete() {
        AutoRunner.log("Current Angle", currentAngle);
        AutoRunner.log("Angle to turn", angleToTurn);
        return Math.abs(currentAngle - angleToTurn) < DEGREES_ERROR_RANGE;
    }

    @Override
    protected void insideRun() {
        currentAngle = robot.imu.getAngularOrientation().firstAngle;
        double error = (angleToTurn + initialAngle) - currentAngle;
        double errorCorrectedPower = (error * kP) + kP * (error > 0 ? 1 : -1);
        robot.drive(0, 0, errorCorrectedPower * power);
    }

    @Override
    protected void onEndRun() {
        robot.stop();
    }
}
