package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * ExampleAuto is an example of an autonomous OpMode using AutoRunner.
 *
 * @author Trinity Chung
 * @version 0.0
 */
@Autonomous(name = "ExampleAuto", group = "TeamCode")
//@Disabled
public class ExampleAutoOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        AutoRunner auto = new AutoRunner("Example", this);
        waitForStart();
        auto.run();
    }

}


