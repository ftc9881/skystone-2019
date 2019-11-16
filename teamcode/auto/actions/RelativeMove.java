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
    private Pose drivePose;


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
        AutoRunner.log("TargetClicks", targetClicks);

        drivePose = new Pose(0, 0, 0);
        drivePose.x = -Math.cos(angle.getRadians());
        drivePose.y = Math.sin(angle.getRadians());
        AutoRunner.log("DrivePowerX", drivePose.x);
        AutoRunner.log("DrivePowerY", drivePose.y);

        this.pidController = new PIDController(0.1, 0, 0, robot.getImuHeading().getRadians());
    }


    @Override
    protected boolean runIsComplete() {
        // Try checking only one wheel
        double actualClicks = Math.abs(robot.driveTrain.lf.getCurrentPosition());
        return Math.abs(actualClicks - targetClicks) < CLICKS_ERROR_RANGE;

    }

    @Override
    protected void insideRun() {
        double actualValue = robot.getImuHeading().getRadians();
        drivePose.r = pidController.getCorrectedOutput(actualValue);
        robot.driveTrain.drive(drivePose, powerFactor);
    }

    @Override
    protected void onEndRun() {
        robot.driveTrain.stop();
    }

}
