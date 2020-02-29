package org.firstinspires.ftc.teamcode.auto.actions;

import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.endconditions.IWatchableDistance;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.SomethingBadHappened;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.hardware.motor.OdometryWheel;
import org.firstinspires.ftc.teamcode.hardware.sensor.IDistanceSensor;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.math.GeneralMath.Conditional;
import org.firstinspires.ftc.teamcode.math.MoveController;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;

public class MoveWithSensorAndOdometry extends Action implements IWatchableDistance {

    private PIDController xPid;
    private MoveController yPid;
    private PIDController rPid;
    private Conditional conditionalY;
    private double closeY;
    private Conditional conditionalX;
    private double closeX;

    private Pose targetPose = new Pose();
    private Pose pose = new Pose();

    private Robot robot;
    private BatMobile batMobile;
    private Command command;
    private Pose drivePose;
    private double basePower;
    private Angle moveAngle;
    private double powerFactor;

    private MovingStatistics sensorReadings;
    private int sensorReadingsSize;
    private double deltaThreshold;
    private double currentSensorReading;

    private OdometryWheel odometryWheel;
    private IDistanceSensor sideSensor;

    private MovingStatistics odometryReadings;
    private int odometryStuckSize;
    private double previousSensorReading;
    private double validXRange;

    public MoveWithSensorAndOdometry(Command command) {
        tag = "MoveWithSensorAndOdometry";
        this.command = command;

        basePower = command.getDouble("base power", 0.3);
        moveAngle = command.getAngle("move angle", 0);
        powerFactor = command.getDouble("power", 0.5);

        robot = Robot.getInstance();
        batMobile = BatMobile.getInstance();
        odometryWheel = batMobile.odometryY;

        sideSensor = batMobile.getSideSensor();

        closeY = command.getDouble("y close threshold", 0.5);
        String conditionalString = command.getString("y stop when", "close");
        conditionalY = Conditional.convertString(conditionalString);

        closeX = command.getDouble("x close threshold", 1);
        conditionalX = Conditional.CLOSE;

        targetPose.x = command.getDouble("target x", 0);
        targetPose.y = command.getDouble("target y", 0);
        targetPose.r = command.getAngle("target r", 0).getDegrees();

        deltaThreshold = command.getDouble("x delta threshold", 5);

        odometryStuckSize = command.getInt("odometry stuck size", 5);
        odometryReadings = new MovingStatistics(odometryStuckSize);

        validXRange = command.getDouble("x reading range", 20);
        sensorReadingsSize = command.getInt("sensor statistics size", 3);
        sensorReadings = new MovingStatistics(sensorReadingsSize);

        currentSensorReading = sideSensor.getDistance();
    }

    public MoveWithSensorAndOdometry(Command command, VisionSystem.SkystonePosition skystonePosition) {
        this(command);
        targetPose.y = command.getDouble("target y " + skystonePosition.key, targetPose.y);
    }

    @Override
    protected void onRun() {
        drivePose = new Pose(0, 0, 0);
        drivePose.x = Math.sin(moveAngle.getRadians());
        drivePose.y = Math.cos(moveAngle.getRadians());

        xPid = new PIDController(command, "x", targetPose.x);
        yPid = new MoveController(command, "y", targetPose.y);
        rPid = new PIDController(command, "r", targetPose.r);

        pose.y = odometryWheel.getInches();
        previousSensorReading = targetPose.x;

        AutoRunner.log("TargetPose", targetPose.toString("\t"));
    }

    @Override
    protected void insideRun() throws SomethingBadHappened {
        Pose correctedDrivePose = new Pose(drivePose);

        currentSensorReading = sideSensor.getDistance();
        AutoRunner.log("Sensor", sideSensor.getDistance());

        if (!batMobile.sensorsAreWorking()) {
            throw new SomethingBadHappened("Uh oh, side sensor is dead");
        }
        else if (Math.abs(targetPose.x - currentSensorReading) < validXRange && sensorReadings.getCount() < sensorReadingsSize) {
            sensorReadings.add(currentSensorReading);
            pose.x = targetPose.x;
        } else if (Math.abs(currentSensorReading - previousSensorReading) < deltaThreshold) {
            previousSensorReading = currentSensorReading;
            sensorReadings.add(currentSensorReading);
            if (sensorReadings.getCount() >= sensorReadingsSize) {
                pose.x = sensorReadings.getMean();
            }
        } else {
            pose.x = targetPose.x;
        }

        pose.y = odometryWheel.getInches();
        pose.r = robot.imu.getHeading().getDegrees();

        odometryReadings.add(pose.y);

        correctedDrivePose.x += xPid.getCorrectedOutput(pose.x);
        correctedDrivePose.y = GeneralMath.clipPower(yPid.getCorrectedOutput(pose.y), basePower) * powerFactor;
        correctedDrivePose.r = rPid.getCorrectedOutput(pose.r);

        robot.driveTrain.drive(correctedDrivePose);

        AutoRunner.log("DrivePose   ", correctedDrivePose.toString("\t"));
        AutoRunner.log("LocationPose", pose.toString("\t"));
    }

    @Override
    protected boolean runIsComplete() {
        boolean odometryIsStuck = odometryReadings.getCount() >= odometryStuckSize && odometryReadings.getStandardDeviation() == 0;
        return ( odometryIsStuck || conditionalY.evaluate(pose.y, targetPose.y, closeY) ) && conditionalX.evaluate(pose.x, targetPose.x, closeX);
    }

    @Override
    protected void onEndRun() {
        robot.driveTrain.stop();
    }

    @Override
    public double getDistance() {
        return odometryWheel.getInches();
    }

}
