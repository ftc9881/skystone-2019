package org.firstinspires.ftc.teamcode.auto.actions;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.hardware.motor.OdometryWheel;
import org.firstinspires.ftc.teamcode.hardware.sensor.IDistanceSensor;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.GeneralMath.Conditional;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;

public class MoveWithSensorAndOdometry extends Action {

    private PIDController xPid;
    private PIDController yPid;
    private PIDController rPid;
    private Conditional conditionalY;
    private double closeY;
    private Conditional conditionalX;
    private double closeX;

    private Pose targetPose = new Pose();
    private Pose pose = new Pose();

    private Robot robot;
    private Command command;
    private Pose drivePose;
    private double basePower;
    private Angle moveAngle;
    private double powerFactor;

    private OdometryWheel odometryWheel;
    private IDistanceSensor frontSensor;
    private IDistanceSensor sideSensor;

    public MoveWithSensorAndOdometry(Command command) {
        tag = "MoveWithSensorAndOdometry";
        this.command = command;

        basePower = command.getDouble("base power", 0.3);
        moveAngle = command.getAngle("move angle", 0);
        powerFactor = command.getDouble("power", 0.5);

        robot = Robot.getInstance();
        BatMobile batMobile = BatMobile.getInstance();
        odometryWheel = batMobile.odometryY;
        frontSensor = batMobile.frontSensor;
        sideSensor= batMobile.getSideSensor();

        closeY = command.getDouble("y close threshold", 4);
        String conditionalString = command.getString("y stop when", "close");
        conditionalY = Conditional.convertString(conditionalString);

        closeX = command.getDouble("x close threshold", 4);
        conditionalX = Conditional.CLOSE;

        targetPose.x = command.getDouble("target x", 0);
        targetPose.y = command.getDouble("target y", 0);
        targetPose.r = command.getAngle("target r", 0).getDegrees();
        AutoRunner.log("TargetPose", targetPose.toString("\t"));
    }

    public MoveWithSensorAndOdometry(Command command, VisionSystem.SkystonePosition skystonePosition) {
        this(command);
        targetPose.y = command.getDouble("y " + skystonePosition.key, targetPose.y);
    }

    @Override
    protected void onRun() {
        drivePose = new Pose(0, 0, 0);
        drivePose.x = Math.sin(moveAngle.getRadians());
        drivePose.y = Math.cos(moveAngle.getRadians());

        xPid = new PIDController(command, "x", targetPose.x);
        yPid = new PIDController(command, "y", targetPose.y);
        rPid = new PIDController(command, "r", targetPose.r);
    }

    @Override
    protected boolean runIsComplete() {
        return conditionalY.evaluate(pose.y, targetPose.y, closeY) && conditionalX.evaluate(pose.x, targetPose.x, closeX);
    }

    @Override
    protected void insideRun() {
        Pose correctedDrivePose = new Pose(drivePose);

//        pose.x = sideSensor.getDistance();
        pose.y = odometryWheel.getInches();
        pose.r = robot.imu.getHeading().getDegrees();

        correctedDrivePose.x += xPid.getCorrectedOutput(pose.x);
        correctedDrivePose.y = yPid.getCorrectedOutput(pose.y);
        correctedDrivePose.r = rPid.getCorrectedOutput(pose.r);

        robot.driveTrain.drive(correctedDrivePose, powerFactor);

        AutoRunner.log("DrivePose", correctedDrivePose.toString("\t"));
        AutoRunner.log("GuessPose", pose.toString("\t"));
    }

    @Override
    protected void onEndRun() {
        robot.driveTrain.stop();
    }

}
