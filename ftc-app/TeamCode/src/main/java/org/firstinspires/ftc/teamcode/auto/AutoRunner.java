package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.actions.ExtendElevatorArm;
import org.firstinspires.ftc.teamcode.auto.actions.LiftElevator;
import org.firstinspires.ftc.teamcode.auto.actions.RelativeMove;
import org.firstinspires.ftc.teamcode.auto.actions.AbsoluteTurn;
import org.firstinspires.ftc.teamcode.auto.endConditions.ObstacleDetect;
import org.firstinspires.ftc.teamcode.auto.endConditions.LookFor;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.CombinedConditions;
import org.firstinspires.ftc.teamcode.auto.vision.OpenCV;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.auto.structure.Command;
import org.firstinspires.ftc.teamcode.auto.structure.AutoOpConfiguration;
import org.firstinspires.ftc.teamcode.auto.structure.IEndCondition;
import org.firstinspires.ftc.teamcode.auto.actions.OdometryMove;
import org.firstinspires.ftc.teamcode.auto.endConditions.Timeout;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.List;

import static android.os.SystemClock.sleep;

/**
 * AutoRunner is the root for all autonomous OpModes.
 * This will read a set of commands from a text configuration file
 * and execute them in order.
 */
public class AutoRunner {

    private static final String TAG_PREFIX = "TeamCode@";
    private static final String TAG = "AutoRunner";
    private static final long SLEEP_LOOP_TIME = 20;

    private AutoOpConfiguration config;
    private LinearOpMode opMode;
    private Robot robot;

    private static AngleUnit angleUnit = AngleUnit.DEGREES;
    private boolean isStopped = false;

    private VisionSystem.SkystonePosition skystonePosition = VisionSystem.SkystonePosition.NONE;

    public AutoRunner(String name, LinearOpMode opMode) {
        opMode.msStuckDetectStop = 10000;
        this.opMode = opMode;

        config = AutoOpConfiguration.newInstance(name + ".json");

        angleUnit = config.properties.getAngleUnit("angle unit", AngleUnit.DEGREES);
        robot = Robot.newInstance(opMode);
        robot.initializeImu(angleUnit);

        for (Command command : config.initCommands) {
            logAndTelemetry(TAG, "Init Command: " + command.name);
            execute(command);
        }

        opMode.telemetry.addData("Skystone Position", skystonePosition.name());
        logAndTelemetry(TAG, "Ready to run");
    }


    public void run() {
        for (Command command : config.commands) {
            logAndTelemetry(TAG, "Command: " + command.name);
            if (isStopped || !opMode.opModeIsActive()) {
                logAndTelemetry(TAG, "Early stop");
                return;
            }
            execute(command);
        }
    }


    private void execute(Command command) {

        if (command.name.contains("SKIP")) {
            return;
        }

        switch (command.name) {

            case "IDENTIFY SKYSTONE": {
                OpenCV openCV = new OpenCV();
                openCV.initialize();
                openCV.startLook(VisionSystem.TargetType.SKYSTONE);

                List<Integer> foundPositions = new ArrayList<>();
                double stdDev = 999;
                int centerX = 0;
                while (centerX == 0 || stdDev > 5) {
                    Rect foundRect = openCV.detector.foundRectangle();
                    centerX = foundRect.x + foundRect.width / 2;
                    foundPositions.add(centerX);
                    stdDev = GeneralMath.standardDeviation(foundPositions);

                    logAndTelemetry("SkystonePixel", centerX);
                }

                int averageCenterX = (int) GeneralMath.mean(foundPositions);
                int skystoneLeftBound = config.properties.getInt("skystone left", 0);
                int skystoneRightBound = config.properties.getInt("skystone right", 0);
                if (averageCenterX < skystoneLeftBound) {
                    skystonePosition = VisionSystem.SkystonePosition.LEFT;
                } else if (averageCenterX > skystoneRightBound) {
                    skystonePosition = VisionSystem.SkystonePosition.RIGHT;
                } else {
                    skystonePosition = VisionSystem.SkystonePosition.CENTER;
                }

                break;
            }
            case "SKYSTONE MOVE": {
                Angle moveAngle = command.getAngle("move angle", 0, angleUnit);
                Angle targetAngle = command.getAngle("target angle", 0, angleUnit);
                int distance = (skystonePosition.ordinal()-1) * config.properties.getInt("stone distance", 4);
                double timeoutMs = command.getDouble("timeout", 10 * 1000.0);
                double powerFactor = command.getDouble("power", 0.5);
                int accelerateClicks = command.getInt("ramp up", 0);
                int decelerateClicks = command.getInt("ramp down", 0);

                Action relativeMove = new RelativeMove(distance, moveAngle, targetAngle, powerFactor, accelerateClicks, decelerateClicks);
                IEndCondition timeoutCondition = new Timeout(timeoutMs);

                runTask(relativeMove, timeoutCondition);
                break;
            }

            case "MOVE": {
                Angle moveAngle = command.getAngle("move angle", 0, angleUnit);
                Angle targetAngle = command.getAngle("target angle", 0, angleUnit);
                double distance = command.getDouble("distance", 5.0);
                double timeoutMs = command.getDouble("timeout", 10 * 1000.0);
                double powerFactor = command.getDouble("power", 0.5);
                int accelerateClicks = command.getInt("ramp up", 0);
                int decelerateClicks = command.getInt("ramp down", 0);

                Action relativeMove = new RelativeMove(distance, moveAngle, targetAngle, powerFactor, accelerateClicks, decelerateClicks);
                IEndCondition timeoutCondition = new Timeout(timeoutMs);

                runTask(relativeMove, timeoutCondition);
                break;
            }

            case "TURN": {
                Angle angle = command.getAngle("angle", 0, angleUnit);
                double power = command.getDouble("power", 1.0);
                double timeoutMs = command.getDouble("timeout", 5 * 1000.0);
                Action relativeTurn = new AbsoluteTurn(angle, power);
                IEndCondition timeoutCondition = new Timeout(timeoutMs);

                runTask(relativeTurn, timeoutCondition);
                break;
            }

            case "ODOMETRY MOVE": {
                double targetX = command.getDouble("x", 0.0);
                double targetY = command.getDouble("y", 0.0);
                double targetR = command.getDouble("r", 0.0);
                double powerFactor = command.getDouble("power", 0.6);
                double timeoutMs = command.getDouble("timeout", 10 * 1000.0);
                double obstacleDistance = command.getDouble("obstacle distance", 10);

                Pose targetPose = new Pose(targetX, targetY, targetR);
                OdometryMove odometryMove = new OdometryMove(targetPose, powerFactor);

                ObstacleDetect obstacleCondition = new ObstacleDetect(obstacleDistance);
                Timeout timeoutCondition = new Timeout(timeoutMs);
                CombinedConditions conditions = new CombinedConditions();
                conditions
                    .add(timeoutCondition)
                    .add(obstacleCondition);

                runTask(odometryMove, conditions);
                break;
            }


            case "EXTEND ELEVATOR ARM": {
                int clicks = command.getInt("clicks", 0);
                double powerFactor = command.getDouble("power factor", 0.5);
                double timeoutMs = command.getDouble("timeout", 5 * 1000.0);

                ExtendElevatorArm extendElevatorArm = new ExtendElevatorArm(clicks, powerFactor);
                Timeout timeoutCondition = new Timeout(timeoutMs);
                runTask(extendElevatorArm, timeoutCondition);
                break;
            }
            case "LIFT ELEVATOR": {
                int clicks = command.getInt("clicks", 0);
                double powerFactor = command.getDouble("power factor", 0.5);
                double timeoutMs = command.getDouble("timeout", 5 * 1000.0);

                LiftElevator liftElevator = new LiftElevator(clicks, powerFactor);
                Timeout timeoutCondition = new Timeout(timeoutMs);
                runTask(liftElevator, timeoutCondition);
                break;
            }
            case "SLEEP": {
                long timeMs = (long) command.getDouble("time", 1000);
                sleep(timeMs);
                break;
            }

            case "STOP": {
                isStopped = true;
                break;
            }

            default: {
                logAndTelemetry("Command not exist");
                break;
            }
        }
    }


    private void runTask(Action action, IEndCondition endCondition) {
        action.start();
        endCondition.start();

        // If end condition completes before action, then action is stopped.
        // If action completes before end condition, then complete.
        while (!action.isStopped() && !endCondition.isTrue()) {
            sleep(SLEEP_LOOP_TIME);
        }

        action.stop();
        endCondition.stop();

        log(TAG, "Run task completed");
    }



    public static void log(String tag, Object message) {
        RobotLog.dd(TAG_PREFIX + tag, message.toString());
    }

    public static void log(Object message) {
        log("Somewhere", message);
    }

    public static AngleUnit getAngleUnits() {
        return angleUnit;
    }


    public void logAndTelemetry(String tag, Object message) {
        AutoRunner.log(tag, message);
        opMode.telemetry.addData(TAG_PREFIX + tag, message);
        opMode.telemetry.update();
    }
    public void logAndTelemetry(Object message) {
        logAndTelemetry("Somewhere", message);
    }


}
