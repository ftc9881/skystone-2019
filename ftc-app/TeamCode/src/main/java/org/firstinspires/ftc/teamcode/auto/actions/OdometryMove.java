package org.firstinspires.ftc.teamcode.auto.actions;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.math.Pose;


public class OdometryMove extends Action {

    private final double POSITION_THRESHOLD = 1;
    private final double ROTATION_THRESHOLD = 0.3;

    private Robot robot;
    private Pose targetPose;
    private Pose currentPose;
    private double powerFactor;


    public OdometryMove (Pose targetPose, double power) {
        this.robot = Robot.getInstance();
        this.currentPose = robot.currentPose;
        this.targetPose = targetPose;
        this.targetPose.r = AutoRunner.getAngleUnits().toRadians(targetPose.r);
        this.powerFactor = power;
    }

    @Override
    protected void onRun() {
        currentPose = robot.odometrySystem.getPose();
    }

    @Override
    protected boolean runIsComplete() {
        boolean positionIsClose = currentPose.distanceTo(targetPose) < POSITION_THRESHOLD;
        boolean rotationIsClose = Math.abs(currentPose.r - targetPose.r) < ROTATION_THRESHOLD;
        return positionIsClose && rotationIsClose;
    }

    @Override
    public void insideRun() {
        robot.odometrySystem.updatePose();
        currentPose = robot.odometrySystem.getPose();

        Pose errorPose = targetPose.subtract(currentPose);
        Pose drivePose = new Pose(0, 0, 0);

        double absAngleDir = Math.atan(errorPose.y / errorPose.x);
        if (errorPose.x < 0) absAngleDir += Math.PI;

        double relAngleError = absAngleDir + currentPose.r;

        double XYMagnitude = Math.sqrt(Math.pow(errorPose.x, 2) + Math.pow(errorPose.y, 2));

        drivePose.x = Range.clip(-XYMagnitude * Math.cos(relAngleError), -1, 1);
        drivePose.y = Range.clip(XYMagnitude * Math.sin(relAngleError), -1, 1);
        drivePose.r = Range.clip(targetPose.r - currentPose.r, -1, 1);

        robot.driveTrain.drive(drivePose, powerFactor);
        AutoRunner.log(drivePose.toString());
    }

    @Override
    protected void onEndRun() {
        robot.driveTrain.stop();
    }

}
