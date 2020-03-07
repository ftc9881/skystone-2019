package org.firstinspires.ftc.teamcode.auto.actions;

import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.endconditions.IWatchableDistance;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.SomethingBadHappened;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.hardware.motor.OdometryWheel;
import org.firstinspires.ftc.teamcode.hardware.sensor.IDistanceSensor;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.math.GeneralMath.Conditional;
import org.firstinspires.ftc.teamcode.math.MoveController;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.math.PoseConditional;
import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;

public class MoveWithSensorAndOdometry extends Action implements IWatchableDistance {

    static private double sensorBasedSkystoneOffsetX;

    private PIDController xPid;
    private MoveController yPid;
    private PIDController rPid;
    private PoseConditional poseConditional;

    private Pose targetPose = new Pose();
    private Pose pose = new Pose();

    private Robot robot;
    private BatMobile batMobile;
    private Command command;
    private Pose drivePose;
    private double basePower;
    private double moveAngleRadians;
    private double turnPower;
    private double powerFactor;

    private MovingStatistics sensorReadings;
    private int sensorReadingsSize;
    private double deltaThreshold;
    private double currentSensorReading;

    private OdometryWheel odometryWheel;
    private IDistanceSensor sideSensor;

    private MovingStatistics odometryReadings;
    private int odometryStuckSize;
    private double odometryStuckStdDev;
    private double previousSensorReading;
    private double validXRange;

    private double blueOffsetY;
    private double blueOffsetX;

    private boolean collectDataX;
    private MovingStatistics dataXStats;

    public MoveWithSensorAndOdometry(Command command) {
        tag = "MoveWithSensorAndOdometry";
        this.command = command;

        basePower = command.getDouble("base power", 0.3);
        moveAngleRadians = command.getAngle("move angle", 0).getRadians() * AutoRunner.signIfFlipForBlue();
        turnPower = command.getDouble("turn power", 0) * AutoRunner.signIfFlipForBlue();
        powerFactor = command.getDouble("power", 0.5);

        robot = Robot.getInstance();
        batMobile = BatMobile.getInstance();
        odometryWheel = batMobile.odometryY;

        sideSensor = batMobile.getSideSensor();

        blueOffsetY = AutoRunner.getSide() == AutoRunner.Side.BLUE ? command.getDouble("y blue offset", 0) : 0;
        blueOffsetX = AutoRunner.getSide() == AutoRunner.Side.BLUE ? command.getDouble("x blue offset", 0) : 0;
        boolean useSensorBasedSkystoneOffsetX = command.getBoolean("use data x", false);
        targetPose.x = command.getDouble("target x", 0) + blueOffsetX + (useSensorBasedSkystoneOffsetX ? sensorBasedSkystoneOffsetX : 0);
        targetPose.y = command.getDouble("target y", 0) + blueOffsetY;
        targetPose.r = command.getAngle("target r", 0).getDegrees() * AutoRunner.signIfFlipForBlue();
        poseConditional = new PoseConditional(command, targetPose);

        deltaThreshold = command.getDouble("x delta threshold", 5);

        odometryStuckSize = command.getInt("odometry stuck size", 5);
        odometryStuckStdDev = command.getDouble("odometry stuck std dev", 0.015);
        odometryReadings = new MovingStatistics(odometryStuckSize);

        validXRange = command.getDouble("x reading range", 20);
        sensorReadingsSize = command.getInt("sensor statistics size", 2);
        sensorReadings = new MovingStatistics(sensorReadingsSize);

        currentSensorReading = sideSensor.getDistance();

        collectDataX = command.getBoolean("collect data x", false);
        if (collectDataX) {
            int dataSize = command.getInt("data size", 20);
            dataXStats = new MovingStatistics(dataSize);
        }
    }

    public MoveWithSensorAndOdometry(Command command, VisionSystem.SkystonePosition skystonePosition) {
        this(command);
        targetPose.y = command.getDouble("target y " + skystonePosition.key, targetPose.y) + blueOffsetY;
    }

    @Override
    protected void onRun() {
        drivePose = new Pose(0, 0, 0);
        drivePose.x = Math.sin(moveAngleRadians);
        drivePose.y = Math.cos(moveAngleRadians);

        xPid = new PIDController(command, "x", targetPose.x);
        yPid = new MoveController(command, "y", targetPose.y);
        rPid = new PIDController(command, "r", targetPose.r);

        if (xPid.getkP() == 0) {
            poseConditional.conditionalX = Conditional.NONE;
        }

        pose.y = odometryWheel.getInches();
        previousSensorReading = targetPose.x;

        AutoRunner.log("TargetPose", targetPose.toString("\t"));
    }

    @Override
    protected void insideRun() throws SomethingBadHappened {
        Pose correctedDrivePose = new Pose(drivePose);

        currentSensorReading = sideSensor.getDistance();

        if (collectDataX) {
            dataXStats.add(currentSensorReading);
            pose.x = targetPose.x;
        } else if (batMobile.sensorDead()) {
            // TODO: At competition, keep going even if sensor die
            throw new SomethingBadHappened("Uh oh, a sensor is dead");
        } else if (Math.abs(targetPose.x - currentSensorReading) < validXRange && sensorReadings.getCount() < sensorReadingsSize) {
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

        correctedDrivePose.x += xPid.getCorrectedOutput(pose.x) * AutoRunner.signIfFlipForBlue();
        correctedDrivePose.y = GeneralMath.clipPower(yPid.getCorrectedOutput(pose.y), basePower) * powerFactor;
        correctedDrivePose.r = rPid.getCorrectedOutput(pose.r) + turnPower;

        robot.driveTrain.drive(correctedDrivePose);

        AutoRunner.log("Sensor", currentSensorReading);
        AutoRunner.log("DrivePose---", correctedDrivePose.toString("\t"));
        AutoRunner.log("LocationPose", pose.toString("\t"));
    }

    @Override
    protected boolean runIsComplete() {
        boolean odometryIsStuck = odometryReadings.getCount() >= odometryStuckSize && odometryReadings.getStandardDeviation() < odometryStuckStdDev;
        return odometryIsStuck || poseConditional.isMet(pose);
    }

    @Override
    protected void onEndRun() {
        robot.driveTrain.stop();
        if (collectDataX) {
            sensorBasedSkystoneOffsetX = dataXStats.getCount() > 0 ? dataXStats.getMean() : sideSensor.getDistance();
        }
        AutoRunner.log("data x ", sensorBasedSkystoneOffsetX);
    }

    @Override
    public double getDistance() {
        return odometryWheel.getInches();
    }

}
