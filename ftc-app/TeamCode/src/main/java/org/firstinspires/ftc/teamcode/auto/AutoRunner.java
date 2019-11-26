package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.actions.RelativeMove;
import org.firstinspires.ftc.teamcode.auto.actions.AbsoluteTurn;
import org.firstinspires.ftc.teamcode.auto.endConditions.ObstacleDetect;
import org.firstinspires.ftc.teamcode.auto.endConditions.StoneInIntake;
import org.firstinspires.ftc.teamcode.auto.endConditions.LookFor;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.CombinedConditions;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.FoundationGrabber;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.auto.structure.Command;
import org.firstinspires.ftc.teamcode.auto.structure.AutoOpConfiguration;
import org.firstinspires.ftc.teamcode.auto.structure.IEndCondition;
import org.firstinspires.ftc.teamcode.auto.actions.OdometryMove;
import org.firstinspires.ftc.teamcode.auto.endConditions.Timeout;

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
    private OpMode opMode;
    private Robot robot;

    private static AngleUnit angleUnit = AngleUnit.DEGREES;
    private boolean isStopped = false;

    public AutoRunner(String name, LinearOpMode opMode) {
        this.opMode = opMode;

        config = AutoOpConfiguration.newInstance(name + ".json");

        angleUnit = config.properties.getAngleUnit("angle unit", AngleUnit.DEGREES);
        robot = Robot.newInstance(opMode);
        robot.initializeImu(angleUnit);

        boolean initializeVision = config.properties.getBoolean("init vision", false);
        if (initializeVision) {
            robot.visionSystem.initialize();
        }

        logAndTelemetry(TAG, "Ready to run");
    }


    public void run() {
        for (Command command : config.commands) {
            logAndTelemetry(TAG, "Command: " + command.name);
            execute(command);
            if (isStopped) {
                logAndTelemetry(TAG, "Early stop");
                return;
            }
        }
    }


    private void execute(Command command) {

        if (command.name.contains("SKIP")) {
            return;
        }

        switch (command.name) {

            case "MOVE": {
                Angle angle = command.getAngle("angle", 0, angleUnit);
                double distance = command.getDouble("distance", 5.0);
                double timeoutMs = command.getDouble("timeout", 10 * 1000.0);
                double powerFactor = command.getDouble("power", 0.5);

                Action relativeMove = new RelativeMove(distance, angle, powerFactor);
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

            case "SEARCH SKYSTONE": {
                // strafe while looking
                Angle angle = command.getAngle("angle", 0, angleUnit);
                double maxDistance = command.getDouble("distance", 30);
                double timeoutMs = command.getDouble("timeout", 5 * 1000.0);
                double powerFactor = command.getDouble("power", 0.5);

                RelativeMove relativeMove = new RelativeMove(maxDistance, angle, powerFactor);
                LookFor skystoneCondition = new LookFor(Vuforia.TargetType.SKYSTONE);
                Timeout timeoutCondition = new Timeout(timeoutMs);
                CombinedConditions conditions = new CombinedConditions();
                conditions
                    .add(timeoutCondition)
                    .add(skystoneCondition);

                runTask(relativeMove, conditions);
                break;
            }

            case "GRAB STONE": {
                robot.intake.in();

                Angle angle = new Angle(0, angleUnit);
                double maxDistance = command.getDouble("distance", 4);
                double timeoutMs = command.getDouble("timeout", 1 * 1000.0);
                double powerFactor = command.getDouble("power", 0.5);

                RelativeMove relativeMove = new RelativeMove(maxDistance, angle, powerFactor);
                Timeout timeoutCondition = new Timeout(timeoutMs);
                StoneInIntake intakeCondition = new StoneInIntake();
                CombinedConditions conditions = new CombinedConditions();
                conditions
                        .add(timeoutCondition)
                        .add(intakeCondition);

                runTask(relativeMove, conditions);

                robot.intake.stop();
                break;
            }

            case "PLACE STONE": {
                robot.intake.out();
                while (robot.sensorSystem.stoneIsIn()) {
                    sleep(20);
                }
                robot.intake.stop();
                break;
            }

            case "MOVE ARM": {
                Arm.State state;
                String rawStateValue = command.getString("state", "nothing").toUpperCase();
                switch (rawStateValue) {
                    case "OUT": {
                        state = Arm.State.OUT;
                        break;
                    }
                    case "IN": {
                        state = Arm.State.IN;
                        break;
                    }
                    case "UP AND OUT": {
                        state = Arm.State.UP_AND_OUT;
                        break;
                    }
                    case "DOWN AND OUT": {
                        state = Arm.State.OUT_AND_DOWN;
                        break;
                    }
                    default: {
                        state = Arm.State.NOTHING;
                        break;
                    }
                }
                robot.arm.move(state);
                break;
            }

            case "GRAB FOUNDATION": {
                robot.foundationGrabber.set(FoundationGrabber.State.GRABBING);
                break;
            }

            case "RELEASE FOUNDATION": {
                robot.foundationGrabber.set(FoundationGrabber.State.RELEASED);
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
