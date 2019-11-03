package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Pose;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.ActionFactory;
import org.firstinspires.ftc.teamcode.auto.structure.Command;
import org.firstinspires.ftc.teamcode.auto.structure.Configuration;
import org.firstinspires.ftc.teamcode.auto.structure.EndCondition;
import org.firstinspires.ftc.teamcode.auto.structure.actions.RelativeMove;
import org.firstinspires.ftc.teamcode.auto.structure.actions.OdometryMove;
import org.firstinspires.ftc.teamcode.auto.structure.endConditions.Timeout;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;


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

    private String name;
    private LinearOpMode opMode;
    private Configuration config;
    private Robot robot;
    private ActionFactory actionFactory;
    private Odometry odometry;
    private Vuforia vuforia;
    private Pose pose;

    public AutoRunner(String name, LinearOpMode opMode) {
        this.name = name;
        this.opMode = opMode;

        robot = new Robot(opMode);
        config = new Configuration(name + ".json");
        actionFactory = new ActionFactory(robot);

        int length = config.commands.size();
        robot.log(Integer.toString(length));
    }

    public void runTask(Action action, EndCondition endCondition) {
        action.run();
        while (!endCondition.isTrue() && !action.isDone()) {
            sleep(50);
        }
        action.stop();
    }

    public void run() {
        robot.log("Running AutoRunner");

        if (!opMode.opModeIsActive()) {
            throw new RuntimeException(name, new Throwable("OpMode is unexpectedly inactive."));
        }

        for (int i = 0; i < config.commands.size(); i++) {
            Command command = config.commands.get(i);
            robot.log(command.toString());
            execute(command);
        }

        robot.log("Completed AutoRunner");
    }

    public void execute(Command command) {
        switch (command.name) {

            case "VUFORIAORIENT": {
                // update coordinates based on vuforia
                vuforia.startLook(Vuforia.TargetType.PERIMETER);
                while (!vuforia.found()) {
                    sleep(50);
                }
                pose = vuforia.getPose();
            }

            case "MOVE": {
                RelativeMove.Direction direction = command.getDirection("direction", RelativeMove.Direction.FRONT);
                double timeoutMs = command.getDouble("timeout", 0.0);
                double clicksToMove = command.getInt("clicksToMove", 0);

                RelativeMove relativeMove = actionFactory.relativeMove(clicksToMove, direction);
                EndCondition timeoutCondition = new Timeout(timeoutMs);

                runTask(relativeMove, timeoutCondition);
            }

            case "ODOMETRY_MOVE": {
                double targetX = command.getDouble("x", 0.0);
                double targetY = command.getDouble("y", 0.0);
                double targetR = command.getDouble("r", 0.0);
                double powerFactor = command.getDouble("power", 0.6);
                double timeoutMs = command.getDouble("timeout", 1.0);

                OdometryMove odometryMove = actionFactory.odometryMove(targetX, targetY, targetR, powerFactor);
                EndCondition timeoutCondition = new Timeout(timeoutMs);

                runTask(odometryMove, timeoutCondition);
            }

            case "LOOKSKYSTONE": {
                // strafe while looking
                // while loop:
                //     isTrue if vuforia found
                //     if found, stop moving
                //     if timeout, stop moving
                // skystone position = something

//                    robot.move();
                vuforia.startLook(Vuforia.TargetType.SKYSTONE);
                while (!vuforia.found()) {
                    sleep(50);
                }

//                    robot.stop();
                vuforia.stopLook();
                robot.log(vuforia.getPose().toString(), true);
            }

            case "GETSKYSTONE": {
                // move arm and pick up skystone
            }

            case "FORKLIFT": {
                // pickup or putdown, which is just toggling the servos
                boolean pickup = command.getBoolean("pickup", false);
                if (pickup) {
                    // move servo
                } else {
                    // move servo
                }
            }

            case "STARTPUTSTONE": {
                // begin putting stone on foundation
                // move some servos

            }

            case "WAITPUTSTONE": {
                // wait for until stone placing is done
            }

        }

    }
}
