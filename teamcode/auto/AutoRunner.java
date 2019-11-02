package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Pose;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.Actions.Action;
import org.firstinspires.ftc.teamcode.auto.utility.Command;
import org.firstinspires.ftc.teamcode.auto.utility.Configuration;
import org.firstinspires.ftc.teamcode.auto.utility.EndCondition;
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
    private Odometry odometry;
    private Vuforia vuforia;
    private Pose pose;

    public AutoRunner(String name, LinearOpMode opMode) {
        this.name = name;
        this.opMode = opMode;

        robot = new Robot(opMode);
        config = new Configuration(name + ".json");
//        vuforia = new Vuforia(robot);
        odometry = new Odometry(robot);

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
                // relative movement
                final int CLICKS_PER_INCH = 2000;


            }

            case "ODOMETRY_MOVE": {

//                double targetX = command.getDouble("x", 0.0);
//                double targetY = command.getDouble("y", 0.0);
//                double targetR = command.getDouble("r", 0.0);
//                double powerFactor = command.getDouble("power", 0.6);
//                Action action = new Action(targetR);
//                EndCondition endCondition = new Timeout(laksjdf);
//                runTask(action, endCondition);


                // end goal: runTask(OdometryMove(x, y, z), endCond());
                // move to coordinate on field
                // properties: x, y, rotation
                // Every cycle:
                // 1: get position from odometry
                // 2: feed x, y, rotation to pid
                // 3: set drives to pid output

                double targetX = command.getDouble("x", 0.0);
                double targetY = command.getDouble("y", 0.0);
                double targetR = command.getDouble("r", 0.0);
                double powerFactor = command.getDouble("power", 0.6);

                Pose targetPose = new Pose(targetX, targetY, targetR);

                Pose currentPose = odometry.getPose();

                //variables we'll need to provide values for;
                Pose pastPose = currentPose;

                while ( (currentPose.distanceTo(targetPose) > 1 || Math.abs(currentPose.r - targetPose.r) > 0.3) ) {
//                            && elapsedTime < timeout) {
                    odometry.updatePose();
                    currentPose = odometry.getPose();

                    Pose errorPose = targetPose.subtract(currentPose);
                    Pose drivePose = new Pose(0,0,0);

                    drivePose.r = Range.clip(targetPose.r - currentPose.r, -1, 1);

                    double absAngleDir = Math.atan(errorPose.y / errorPose.x);
                    if (errorPose.x < 0) absAngleDir += Math.PI;

                    double relAngleError = absAngleDir + currentPose.r;

                    double XYMagnitude = Math.sqrt(Math.pow(errorPose.x, 2) + Math.pow(errorPose.y, 2));

                    drivePose.x = Range.clip(-XYMagnitude * Math.cos(relAngleError), -1, 1);
                    drivePose.y = Range.clip(XYMagnitude * Math.sin(relAngleError), -1, 1);

                    // only proportional correction
                    robot.drive(drivePose, powerFactor);
                    opMode.telemetry.addData("CurrentPose", currentPose.toString());
                    robot.log(drivePose.toString());

//                        if (obstacleDetection) {
//                            while (obstacle.stillThere()) {
//                                sleep(50);
//
//                            }
//                        }

                    pastPose = currentPose;
                }
                robot.drive(0, 0, 0);
                robot.log("Done with move");

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
