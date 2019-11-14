package org.firstinspires.ftc.teamcode.auto.actions;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.math.PIDController;

public class RelativeMove extends Action {

    public enum Direction {
        FRONT,
        LEFT,
        RIGHT,
        BACK
    }

    private static final double CLICKS_PER_INCH = 50.0;
    private static final double CLICKS_ERROR_RANGE = 100.0;

    private PIDController pidController;

    Robot robot;
    private double targetClicks;
    private double distance;
    private double angleInRadians;
    private double powerFactor;


    public RelativeMove (Robot robot, double distance, double angleInRadians, double powerFactor) {
        this.robot = robot;
        this.distance = distance;
        this.angleInRadians = angleInRadians;
        this.powerFactor = powerFactor;
    }


    @Override
    protected void onRun() {
        robot.driveTrain.resetEncoders();
        // TODO: Right now this is an arbitrary distance
        targetClicks = distance * CLICKS_PER_INCH;

        AutoRunner.log("Target Position", targetClicks);

        this.pidController = new PIDController(0.1, 0, 0, robot.getImuHeading());
    }


    @Override
    protected boolean runIsComplete() {
        // Try checking only one wheel
        return Math.abs(robot.driveTrain.lf.getCurrentPosition() - targetClicks) < CLICKS_ERROR_RANGE;

    }

    @Override
    protected void insideRun() {
        double actualValue = robot.getImuHeading();
        double pidRotationOutput = pidController.getCorrectedOutput(actualValue);
        double forwardsOutput = Math.cos(angleInRadians);
        double rightLeftOutput = Math.sin(angleInRadians);

        // Try without pid
        robot.drive(forwardsOutput, rightLeftOutput, pidRotationOutput, powerFactor);

    }

    @Override
    protected void onEndRun() {
        robot.stop();
    }

}
