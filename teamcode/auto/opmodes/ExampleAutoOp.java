package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;

/**
 * ExampleAuto is an example of an autonomous OpMode using AutoRunner.
 *
 * @author Trinity Chung
 * @version 1.0
 */
@Autonomous(name = "ExampleAuto", group = "TeamCode")
@Disabled
public class ExampleAutoOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        AutoRunner auto = new AutoRunner("Example", this);
        waitForStart();
        auto.run();
    }

}


