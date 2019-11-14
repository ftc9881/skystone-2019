package org.firstinspires.ftc.teamcode.auto.actions;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.math.PIDController;

public class RelativeMove extends Action {

    private static final double CLICKS_PER_INCH = 50.0;
    private static final double CLICKS_ERROR_RANGE = 100.0;

    private PIDController pidController;

    Robot robot;
    private double targetClicks;
    private double distance;
    private Angle angle;
    private double powerFactor;


    public RelativeMove (double distance, Angle angle, double powerFactor) {
        this.robot = Robot.getInstance();
        this.distance = distance;
        this.angle = angle;
        this.powerFactor = powerFactor;
    }


    @Override
    protected void onRun() {
        robot.driveTrain.resetEncoders();
        // TODO: Right now this is an arbitrary distance
        targetClicks = distance * CLICKS_PER_INCH;

        AutoRunner.log("Target Position", targetClicks);

        this.pidController = new PIDController(0.1, 0, 0, robot.getImuHeading().getRadians());
    }


    @Override
    protected boolean runIsComplete() {
        // Try checking only one wheel
        return Math.abs(robot.driveTrain.lf.getCurrentPosition() - targetClicks) < CLICKS_ERROR_RANGE;

    }

    @Override
    protected void insideRun() {
        double actualValue = robot.getImuHeading().getRadians();
        double pidRotationOutput = pidController.getCorrectedOutput(actualValue);
        double forwardsOutput = Math.cos(angle.getRadians());
        double rightLeftOutput = Math.sin(angle.getRadians());
        Pose drivePose = new Pose(forwardsOutput, rightLeftOutput, pidRotationOutput);
        robot.driveTrain.drive(drivePose, powerFactor);
    }

    @Override
    protected void onEndRun() {
        robot.driveTrain.stop();
    }

}
