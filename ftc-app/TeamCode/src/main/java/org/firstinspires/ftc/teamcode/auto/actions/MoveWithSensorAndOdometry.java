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
    private double moveAngleRadians;
    private double powerFactor;

    private MovingStatistics sensorReadings;
    private int sensorReadingsSize;
    private double deltaThreshold;
    private double currentSensorReading;

    private OdometryWheel odometryWheel;
    private IDistanceSensor sideSensor;
    private IDistanceSensor skystoneSideSensor;
    private IDistanceSensor frontSensor;

    private MovingStatistics odometryReadings;
    private int odometryStuckSize;
    private double previousSensorReading;
    private double validXRange;

    private boolean useFrontSensor;
    private boolean doCorrectX;

    private double blueOffsetY;
    private double blueOffsetX;

    public MoveWithSensorAndOdometry(Command command) {
        tag = "MoveWithSensorAndOdometry";
        this.command = command;

        basePower = command.getDouble("base power", 0.3);
        moveAngleRadians = command.getAngle("move angle", 0).getRadians() * AutoRunner.signIfFlipForBlue();
        powerFactor = command.getDouble("power", 0.5);

        robot = Robot.getInstance();
        batMobile = BatMobile.getInstance();
        odometryWheel = batMobile.odometryY;

        sideSensor = batMobile.getSideSensor();
        skystoneSideSensor = batMobile.getOtherSideSensor();
        frontSensor = batMobile.frontSensor;
        useFrontSensor = command.getBoolean("use front sensor", false);
//        useSkystoneSensor = command.getBoolean("use skystone sensor", false);

        closeY = command.getDouble("y close threshold", 0.5);
        String conditionalString = command.getString("y stop when", "close");
        conditionalY = Conditional.convertString(conditionalString);

        closeX = command.getDouble("x close threshold", 1);
        conditionalX = Conditional.CLOSE;


        blueOffsetY = AutoRunner.getSide() == AutoRunner.Side.BLUE ? command.getDouble("y blue offset", 0) : 0;
        blueOffsetX = AutoRunner.getSide() == AutoRunner.Side.BLUE ? command.getDouble("x blue offset", 0) : 0;
        targetPose.x = command.getDouble("target x", 0) + blueOffsetX;
        targetPose.y = command.getDouble("target y", 0) + blueOffsetY;
        targetPose.r = command.getAngle("target r", 0).getDegrees() * AutoRunner.signIfFlipForBlue();

        deltaThreshold = command.getDouble("x delta threshold", 5);

        odometryStuckSize = command.getInt("odometry stuck size", 5);
        odometryReadings = new MovingStatistics(odometryStuckSize);

        validXRange = command.getDouble("x reading range", 20);
        sensorReadingsSize = command.getInt("sensor statistics size", 2);
        sensorReadings = new MovingStatistics(sensorReadingsSize);

        currentSensorReading = sideSensor.getDistance();
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

        doCorrectX = xPid.getkP() != 0;

        pose.y = getY();
        previousSensorReading = targetPose.x;

        AutoRunner.log("TargetPose", targetPose.toString("\t"));
    }

    @Override
    protected void insideRun() throws SomethingBadHappened {
        Pose correctedDrivePose = new Pose(drivePose);

        currentSensorReading = sideSensor.getDistance();
        AutoRunner.log("Sensor", currentSensorReading);

        if (batMobile.sensorDead()) {
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

        pose.y = getY();
        pose.r = robot.imu.getHeading().getDegrees();

        odometryReadings.add(pose.y);

        correctedDrivePose.x += xPid.getCorrectedOutput(pose.x) * AutoRunner.signIfFlipForBlue();
        correctedDrivePose.y = GeneralMath.clipPower(yPid.getCorrectedOutput(pose.y), basePower) * powerFactor;
        correctedDrivePose.r = rPid.getCorrectedOutput(pose.r);

        robot.driveTrain.drive(correctedDrivePose);

        AutoRunner.log("DrivePose---", correctedDrivePose.toString("\t"));
        AutoRunner.log("LocationPose", pose.toString("\t"));
    }

    private double getY() {
        return useFrontSensor ? frontSensor.getDistance() : odometryWheel.getInches();
    }

    @Override
    protected boolean runIsComplete() {
        boolean odometryIsStuck = odometryReadings.getCount() >= odometryStuckSize && odometryReadings.getStandardDeviation() == 0;
        return odometryIsStuck || (conditionalY.evaluate(pose.y, targetPose.y, closeY) && (conditionalX.evaluate(pose.x, targetPose.x, closeX) || !doCorrectX));
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
