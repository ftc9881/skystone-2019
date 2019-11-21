package org.firstinspires.ftc.teamcode.auto.actions;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.AutoOpConfiguration;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.math.PIDController;

@Deprecated
public class ArbitraryRelativeMove extends Action {

    private static final double CLICKS_PER_INCH = 50.0;
    private static final double CLICKS_ERROR_RANGE = 100.0;

    private PIDController pidController;

    Robot robot;
    private double targetClicks;
    private double distance;
    private Angle angle;
    private double powerFactor;
    private Pose drivePose;


    public ArbitraryRelativeMove(double distance, Angle angle, double powerFactor) {
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
        drivePose.x = Math.cos(angle.getRadians());
        drivePose.y = Math.sin(angle.getRadians());
        AutoRunner.log("DrivePowerX", drivePose.x);
        AutoRunner.log("DrivePowerY", drivePose.y);

        AutoOpConfiguration config = AutoOpConfiguration.getInstance();
        double kP = config.properties.getDouble("move kp", 0);
        double kI = config.properties.getDouble("move ki", 0);
        double kD = config.properties.getDouble("move kd", 0);
        this.pidController = new PIDController(kP, kI, kD, robot.getImuHeading().getRadians());
    }

    @Override
    protected boolean runIsComplete() {
        // TODO: currently this is an arbitrary check on all wheels; want to calculate actual
        double actualClicksLF = Math.abs(robot.driveTrain.lf.getCurrentPosition());
        double actualClicksLB = Math.abs(robot.driveTrain.lb.getCurrentPosition());
        double actualClicksRF = Math.abs(robot.driveTrain.rf.getCurrentPosition());
        double actualClicksRB = Math.abs(robot.driveTrain.rb.getCurrentPosition());
        boolean lfReached = Math.abs(actualClicksLF - targetClicks) < CLICKS_ERROR_RANGE;
        boolean lbReached = Math.abs(actualClicksLB - targetClicks) < CLICKS_ERROR_RANGE;
        boolean rfReached = Math.abs(actualClicksRF - targetClicks) < CLICKS_ERROR_RANGE;
        boolean rbReached = Math.abs(actualClicksRB - targetClicks) < CLICKS_ERROR_RANGE;

        return lfReached || lbReached || rfReached || rbReached;
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
