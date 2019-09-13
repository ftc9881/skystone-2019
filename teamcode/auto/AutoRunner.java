package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
                robot.log("JSONTEST", command.toString());
                execute(command);
            }

        } catch (JSONException e) {
            robot.log("JSON", e.getMessage());
        }


        robot.log(name, "Completed AutoRunner");
    }

    public void execute(JSONObject command) {
        try {
            switch (command.getString("command")) {
                case "test": {
                    robot.log("JSONTEST", "in execute", true);
                    android.os.SystemClock.sleep(5000);
                }
            }
        } catch (JSONException e) {
            robot.log("JSON", e.getMessage());
        }

    }
}
