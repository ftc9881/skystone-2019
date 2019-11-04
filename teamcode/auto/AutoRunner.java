package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.endConditions.SkystoneRecognition;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.EndConditionFactory;
import org.firstinspires.ftc.teamcode.utility.Pose;
import org.firstinspires.ftc.teamcode.Robot;
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
 * @version 0.0
 */
public class AutoRunner {

    private static final long SLEEP_LOOP_TIME = 20;

    private String name;
    private LinearOpMode opMode;
    private Configuration config;
    private Robot robot;
    private ActionFactory actionFactory;
    private EndConditionFactory conditionFactory;
    private Pose pose;


    public AutoRunner(String name, LinearOpMode opMode) {
        this.name = name;
        this.opMode = opMode;

        robot = new Robot(opMode);
        config = new Configuration(name + ".json");
        actionFactory = new ActionFactory(robot);
        conditionFactory = new EndConditionFactory(robot);
    }


    public void run() {
        for (Command command : config.commands) {
            robot.log(command.toString());
            execute(command);
        }
    }


    private void execute(Command command) {
        switch (command.name) {

            case "VUFORIA ORIENT": {
                // update coordinates based on vuforia


//                vuforia.startLook(Vuforia.TargetType.PERIMETER);
//                while (!vuforia.found()) {
//                    sleep(50);
//                }
//                pose = vuforia.getPose();
                break;
            }

            case "MOVE": {
                Direction direction = command.getDirection("direction", Direction.FRONT);
                double timeoutMs = command.getDouble("timeout", 0.0);
                double clicksToMove = command.getInt("clicksToMove", 0);

                RelativeMove relativeMove = actionFactory.relativeMove(clicksToMove, direction);
                IEndCondition timeoutCondition = new Timeout(timeoutMs);

                runTask(relativeMove, timeoutCondition);
                break;
            }

            case "ODOMETRY_MOVE": {
                double targetX = command.getDouble("x", 0.0);
                double targetY = command.getDouble("y", 0.0);
                double targetR = command.getDouble("r", 0.0);
                Pose targetPose = new Pose(targetX, targetY, targetR);
                double powerFactor = command.getDouble("power", 0.6);
                double timeoutMs = command.getDouble("timeout", 10 * 1000.0);

                OdometryMove odometryMove = actionFactory.odometryMove(targetPose, powerFactor);
                Timeout timeoutCondition = conditionFactory.timeout(timeoutMs);

                runTask(odometryMove, timeoutCondition);
                pose = odometryMove.getPose();
                break;
            }

            case "SEARCH SKYSTONE": {
                // strafe while looking
                RelativeMove relativeMove = actionFactory.relativeMove(99, Direction.LEFT);
                SkystoneRecognition skystoneCondition = conditionFactory.skystoneRecognition();
                runTask(relativeMove, skystoneCondition);
                break;
            }

            case "GET SKYSTONE": {
                // move arm and pick up skystone
                break;
            }

            case "GRAB":
            case "GRAB FOUNDATION": {
                robot.foundationGrab();
                break;
            }

            case "RELEASE":
            case "RELEASE FOUNDATION": {
                robot.foundationRelease();
                break;
            }

            case "PLACE STONE": {
                // begin putting stone on foundation
                // move some servos
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

        robot.log("task completed");
    }

}
