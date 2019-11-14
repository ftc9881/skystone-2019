package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.auto.endConditions.ObstacleDetect;
import org.firstinspires.ftc.teamcode.auto.endConditions.VuforiaLook;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.CombinedConditions;
import org.firstinspires.ftc.teamcode.auto.structure.EndConditionFactory;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.auto.structure.ActionFactory;
import org.firstinspires.ftc.teamcode.auto.structure.Command;
import org.firstinspires.ftc.teamcode.auto.structure.Configuration;
import org.firstinspires.ftc.teamcode.auto.structure.IEndCondition;
import org.firstinspires.ftc.teamcode.auto.actions.RelativeMove;
import org.firstinspires.ftc.teamcode.auto.actions.OdometryMove;
import org.firstinspires.ftc.teamcode.auto.endConditions.Timeout;
import org.firstinspires.ftc.teamcode.auto.actions.RelativeMove.Direction;

import static android.os.SystemClock.sleep;

/**
 * AutoRunner is the root for all autonomous OpModes.
 * This will read a set of commands from a text configuration file
 * and execute them in order.
 *
 * @author Trinity Chung
 * @version 1.0
 */
public class AutoRunner {

    private static final String TAG_PREFIX = "TeamCode@";
    private static final String TAG = "AutoRunner";
    private static final long SLEEP_LOOP_TIME = 20;

    private String name;
    private Configuration config;
    private OpMode opMode;
    private Robot robot;
    private ActionFactory actionFactory;
    private EndConditionFactory conditionFactory;


    public AutoRunner(String name, LinearOpMode opMode) {
        this.name = name;
        this.opMode = opMode;

        robot = new Robot(opMode);
        config = new Configuration(name + ".json");
        actionFactory = new ActionFactory(robot);
        conditionFactory = new EndConditionFactory(robot);
    }

    public void initialize() {
//        config.properties.getBoolean("vuforia");
    }


    public void run() {
        for (Command command : config.commands) {
            logAndTelemetry(name, command.name);

            if (command.name == "STOP") {
                break;
            }

            execute(command);
        }
    }


    private void execute(Command command) {
        switch (command.name) {

            case "MOVE": {
                double distance = command.getDouble("distance", 5.0);
                double angle = command.getDouble("angle", 0);
                double timeoutMs = command.getDouble("timeout", 10 * 1000.0);
                double powerFactor = command.getDouble("power", 0.5);

                Action relativeMove = actionFactory.relativeMove(distance, angle, powerFactor);
                IEndCondition timeoutCondition = conditionFactory.timeout(timeoutMs);

                runTask(relativeMove, timeoutCondition);
                break;
            }

            case "TURN": {
                double angle = command.getDouble("angle", 0.0);
                double power = command.getDouble("power", 1.0);
                double timeoutMs = command.getDouble("timeout", 5 * 1000.0);
                Action relativeTurn = actionFactory.relativeTurn(angle, power);
                Timeout timeoutCondition = conditionFactory.timeout(timeoutMs);

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
                OdometryMove odometryMove = actionFactory.odometryMove(targetPose, powerFactor);

                ObstacleDetect obstacleCondition = conditionFactory.obstacleDetect(obstacleDistance);
                Timeout timeoutCondition = conditionFactory.timeout(timeoutMs);
                CombinedConditions conditions = new CombinedConditions();
                conditions
                    .add(timeoutCondition)
                    .add(obstacleCondition);

                runTask(odometryMove, conditions);
                break;
            }

            case "VUFORIA ORIENT": {
                // strafe while looking
                double maxDistance = command.getDouble("distance", 30);
                double timeoutMs = command.getDouble("timeout", 10 * 1000.0);
                double angle = command.getDouble("angle", 0);
                double powerFactor = command.getDouble("power", 0.5);

                RelativeMove relativeMove = actionFactory.relativeMove(maxDistance, angle, powerFactor);
                VuforiaLook vuforiaCondition = conditionFactory.vuforiaLook(Vuforia.TargetType.PERIMETER);
                Timeout timeoutCondition = conditionFactory.timeout(timeoutMs);
                CombinedConditions conditions = new CombinedConditions();
                conditions
                    .add(timeoutCondition)
                    .add(vuforiaCondition);

                runTask(relativeMove, conditions);
                robot.currentPose = vuforiaCondition.getPose();
                break;
            }

            case "SEARCH SKYSTONE": {
                // strafe while looking
                double maxDistance = command.getDouble("distance", 30);
                double timeoutMs = command.getDouble("timeout", 5 * 1000.0);
                double angle = command.getDouble("angle", 0);
                double powerFactor = command.getDouble("power", 0.5);

                RelativeMove relativeMove = actionFactory.relativeMove(maxDistance, angle, powerFactor);
                VuforiaLook skystoneCondition = conditionFactory.vuforiaLook(Vuforia.TargetType.SKYSTONE);
                Timeout timeoutCondition = conditionFactory.timeout(timeoutMs);
                CombinedConditions conditions = new CombinedConditions();
                conditions
                    .add(timeoutCondition)
                    .add(skystoneCondition);

                runTask(relativeMove, conditions);
                break;
            }

            case "GRAB STONE": {
                robot.intake.in();

                double maxDistance = command.getDouble("distance", 4);
                double timeoutMs = command.getDouble("timeout", 1 * 1000.0);
                double powerFactor = command.getDouble("power", 0.5);

                RelativeMove relativeMove = actionFactory.relativeMove(maxDistance, 0, powerFactor);
                Timeout timeoutCondition = conditionFactory.timeout(timeoutMs);
                // TODO: Add StoneInIntake condition once Michael gets his switch thingy

                runTask(relativeMove, timeoutCondition);

                robot.intake.stop();
                break;
            }

            case "PLACE STONE": {
                // TODO: Implement
//                robot.arm.placeStone();
                break;
            }

            case "GRAB FOUNDATION": {
                robot.foundationGrabber.grab();
                break;
            }

            case "RELEASE FOUNDATION": {
                robot.foundationGrabber.release();
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


    public void logAndTelemetry(String tag, Object message) {
        AutoRunner.log(tag, message);
        opMode.telemetry.addData("TAG_PREFIX" + tag, message);
        opMode.telemetry.update();
    }
    public void logAndTelemetry(Object message) {
        logAndTelemetry("Somewhere", message);
    }


}
