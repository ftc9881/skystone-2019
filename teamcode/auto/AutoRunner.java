package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Position;
import org.firstinspires.ftc.teamcode.auto.utility.Configuration;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

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

    public AutoRunner(String name, LinearOpMode opMode) {
        this.name = name;
        this.opMode = opMode;

        robot = new Robot(opMode);
        config = new Configuration(name + ".json");
        vuforia = new Vuforia(robot);
    }

    public void run() {
        robot.log(name, "Running AutoRunner");

        if (!opMode.opModeIsActive()) {
            throw new RuntimeException(name, new Throwable("OpMode is unexpectedly inactive."));
        }

        try {

            for (int i = 0; i < config.commands.length(); i++) {
                JSONObject command = config.commands.getJSONObject(i);
                robot.log("AUTO COMMAND", command.toString());
                execute(command);
            }

        } catch (JSONException e) {
            robot.log("JSON", e.getMessage());
        }


        robot.log(name, "Completed AutoRunner");
    }

    public void execute(JSONObject command) {
        try {
            switch (command.getString("command").toUpperCase()) {

                case "VUFORIAORIENT": {
                    // update coordinates based on vuforia
                }

                case "MOVE": {
                    // move to coordinate on field
                    // properties: x, y, rotation
                    // Every cycle:
                    // 1: get position from odometry
                    // 2: feed x, y, rotation to pid
                    // 3: set drives to pid output

                    /*

                    double targetX = command.getDouble("x");
                    double targetY = command.getDouble("y");
                    double targetR = command.getDouble("r");
                    Position targetPosition = new Position(targetX, targetY, targetR);

                    Position curPosition = odometry.getPosition();

                    //variables we'll need to provide values for;
                    Position prevPosition = curPosition;
                    double prevT;
                    double currT;
                    while (Position.distance(curPosition, targetPosition) < 1) {
                        curPosition = odometry.getPosition();

                        //



                        prevPosition = curPosition;
                    }


                     */

                }

                case "LOOKSKYSTONE": {
                    // strafe while looking
                    // while loop:
                    //     check if vuforia found
                    //     if found, stop moving
                    //     if timeout, stop moving
                    // skystone position = something

//                    robot.move();
                    vuforia.startLook();
                    while (!vuforia.found()) {
                        sleep(50);
                    }

//                    robot.stop();
                    vuforia.stopLook();
                    robot.log("LOOK", vuforia.getPose().toString(), true);
                    sleep(1000);
                }

                case "GETSKYSTONE": {
                    // move arm and pick up skystone
                    // vuforia.getPose();
                }

                case "FORKLIFT": {
                    // pickup or putdown, which is just toggling the servos
                }

                case "STARTPUTSTONE": {
                    // begin putting stone on foundation
                }

                case "WAITPUTSTONE": {
                    // wait for until stone placing is done
                }

                // for debugging
                case "TEST": {
                    robot.log("JSONTEST", "in execute", true);
                    robot.log("JSONTEST",command.getString("a"));
                }

            }
        } catch (JSONException e) {
            robot.log("JSON", e.getMessage());
        }

    }
}
