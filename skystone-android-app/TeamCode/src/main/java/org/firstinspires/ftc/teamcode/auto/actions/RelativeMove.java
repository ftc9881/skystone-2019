package org.firstinspires.ftc.teamcode.auto.actions;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.AutoOpConfiguration;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.XYOdometrySystem;

public class RelativeMove extends Action {

    private static final double CLICKS_PER_INCH = 50.0;
    private static final double CLICKS_ERROR_RANGE = 100.0;

    private PIDController pidController;

    private Robot robot;
    private double targetClicksX;
    private double targetClicksY;
    private double distance;
    private Angle angle;
    private double powerFactor;
    private Pose drivePose;

    public RelativeMove(double distance, Angle angle, double powerFactor) {
        this.robot = Robot.getInstance();
        this.distance = distance;
        this.angle = angle;
        this.powerFactor = powerFactor;
    }


    @Override
    protected void onRun() {
        drivePose = new Pose(0, 0, 0);
        drivePose.x = Math.cos(angle.getRadians());
        drivePose.y = Math.sin(angle.getRadians());
        AutoRunner.log("DrivePowerX", drivePose.x);
        AutoRunner.log("DrivePowerY", drivePose.y);

        robot.xyOdometrySystem.resetEncoders();
        targetClicksX = drivePose.x * distance * CLICKS_PER_INCH;
        targetClicksY = drivePose.y * distance * CLICKS_PER_INCH;
        AutoRunner.log("TargetClicksX", targetClicksX);
        AutoRunner.log("TargetClicksY", targetClicksY);


        AutoOpConfiguration config = AutoOpConfiguration.getInstance();
        double kP = config.properties.getDouble("move kp", 0);
        double kI = config.properties.getDouble("move ki", 0);
        double kD = config.properties.getDouble("move kd", 0);
        this.pidController = new PIDController(kP, kI, kD, robot.getImuHeading().getRadians());
    }

    @Override
    protected boolean runIsComplete() {
        double clicksX = Math.abs(robot.xyOdometrySystem.getX());
        double clicksY = Math.abs(robot.xyOdometrySystem.getY());
        boolean reachedX = Math.abs(targetClicksX - clicksX) < CLICKS_ERROR_RANGE;
        boolean reachedY = Math.abs(targetClicksY - clicksY) < CLICKS_ERROR_RANGE;
        return reachedX || reachedY;
    }

    @Override
    protected void insideRun() {
        // TODO: pid amplifies error?
        double actualValue = robot.getImuHeading().getRadians();
        drivePose.r = pidController.getCorrectedOutput(actualValue);
        robot.driveTrain.drive(drivePose, powerFactor);
    }

    @Override
    protected void onEndRun() {
        robot.driveTrain.stop();
    }

}
