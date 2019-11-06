package org.firstinspires.ftc.teamcode.auto.actions;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.IPoseChanger;
import org.firstinspires.ftc.teamcode.utility.Odometry;
import org.firstinspires.ftc.teamcode.utility.Pose;


public class OdometryMove extends Action implements IPoseChanger {

    private final double POSITION_THRESHOLD = 1;
    private final double ROTATION_THRESHOLD = 0.3;

    private Robot robot;
    private Odometry odometry;
    private Pose targetPose;
    private Pose currentPose;
    private double powerFactor;


    public OdometryMove (Robot robot, Odometry odometry, Pose currentPose, Pose targetPose, double power) {
        this.robot = robot;
        this.odometry = odometry;
        this.currentPose = currentPose;
        this.targetPose = targetPose;
        this.powerFactor = power;
    }

    @Override
    protected void onRun() {
        currentPose = odometry.getPose();
    }

    @Override
    protected boolean runIsComplete() {
        boolean positionIsClose = currentPose.distanceTo(targetPose) < POSITION_THRESHOLD;
        boolean rotationIsClose = Math.abs(currentPose.r - targetPose.r) < ROTATION_THRESHOLD;
        return positionIsClose && rotationIsClose;
    }

    @Override
    public void insideRun() {
        odometry.updatePose();
        currentPose = odometry.getPose();

        Pose errorPose = targetPose.subtract(currentPose);
        Pose drivePose = new Pose(0, 0, 0);

        double absAngleDir = Math.atan(errorPose.y / errorPose.x);
        if (errorPose.x < 0) absAngleDir += Math.PI;

        double relAngleError = absAngleDir + currentPose.r;

        double XYMagnitude = Math.sqrt(Math.pow(errorPose.x, 2) + Math.pow(errorPose.y, 2));

        drivePose.x = Range.clip(-XYMagnitude * Math.cos(relAngleError), -1, 1);
        drivePose.y = Range.clip(XYMagnitude * Math.sin(relAngleError), -1, 1);
        drivePose.r = Range.clip(targetPose.r - currentPose.r, -1, 1);

        robot.drive(drivePose, powerFactor);
        robot.logAndTelemetry(drivePose.toString());
    }

    @Override
    protected void onEndRun() {
        robot.stop();
    }

    @Override
    public Pose getPose() {
        return odometry.getPose();
    }

}
