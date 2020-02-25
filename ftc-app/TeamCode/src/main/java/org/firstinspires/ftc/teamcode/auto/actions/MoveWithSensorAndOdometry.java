package org.firstinspires.ftc.teamcode.auto.actions;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.hardware.motor.OdometryWheel;
import org.firstinspires.ftc.teamcode.hardware.sensor.IDistanceSensor;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.GeneralMath.Conditional;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;

public class MoveWithSensorAndOdometry extends MoveWithClicks {

    private PIDController xPid;
    private PIDController yPid;
    private double targetY;
    private double targetX;
    private Conditional conditionalY;
    private double closeY;
    private Conditional conditionalX;
    private double closeX;

    private Pose pose = new Pose();

    private double jumpThreshold;

    private OdometryWheel odometryWheel;
    private IDistanceSensor frontSensor;
    private IDistanceSensor sideSensor;

    public MoveWithSensorAndOdometry(Command command) {
        super(command);
        tag = "MoveWithSensorAndOdometry";

        jumpThreshold = command.getDouble("jump threshold", 1);

        closeY = command.getDouble("y close threshold", 4);
        String conditionalString = command.getString("y stop when", "close");
        conditionalY = Conditional.convertString(conditionalString);

        closeX = command.getDouble("x close threshold", 4);
        conditionalX = Conditional.CLOSE;

        targetX = command.getDouble("target x", 0);
        xPid = new PIDController(command, "x", targetX);
        targetY = command.getDouble("target y", 0);
        yPid = new PIDController(command, "y", targetY);
    }

    public MoveWithSensorAndOdometry(Command command, VisionSystem.SkystonePosition skystonePosition) {
        this(command);
        targetDistance = command.getDouble("inches " + skystonePosition.key, targetDistance);
        targetY = command.getDouble("y " + skystonePosition.key, targetY);
        yPid = new PIDController(command, "y", targetY);
    }

    @Override
    protected void onRun() {
        super.onRun();
    }

    @Override
    protected boolean runIsComplete() {
        return conditionalY.evaluate(pose.y, targetY, closeY) && conditionalX.evaluate(pose.x, targetX, closeX);
    }

    @Override
    protected void insideRun() {
        Pose correctedDrivePose = new Pose(drivePose);

        pose.x = sideSensor.getDistance();
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
