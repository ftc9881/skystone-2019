package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.utility.Configuration;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

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

    public AutoRunner(String name, LinearOpMode opMode) {
        this.name = name;
        this.opMode = opMode;

        robot = new Robot(opMode);
        config = new Configuration(name + ".json");
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
                }

                case "LOOKSKYSTONE": {
                    // strafe while looking
                    // while loop:
                    //
                    // Vuforia.look()
                    // wait for completion or timeout
                    // skystone position = something
                }

                case "GETSKYSTONE": {
                    // move arm and pick up skystone
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
