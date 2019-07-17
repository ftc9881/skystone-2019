package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.utility.Configuration;

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
    private Configuration configuration;
    private Robot robot;

    public AutoRunner(String name, LinearOpMode opMode) {
        this.name = name;

        robot = new Robot(opMode);
        configuration = new Configuration(name + ".config");
    }

    public void run() {
        robot.log(name, "Run AutoRunner");

        if (!opMode.opModeIsActive()) {
            throw new RuntimeException(name, new Throwable("OpMode is unexpectedly inactive."));
        }


        //TODO: Execute commands in configuration



        robot.log(name, "Completed AutoRunner");
    }
}
