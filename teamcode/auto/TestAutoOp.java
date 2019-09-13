package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;

/**
 * ExampleAuto is an example of an autonomous OpMode using AutoRunner.
 *
 * @author Trinity Chung
 * @version 0.0
 */
@Autonomous(name = "TestAuto", group = "TeamCode")
//@Disabled
public class TestAutoOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        AutoRunner auto = new AutoRunner("Test", this);
        waitForStart();
        auto.run();
    }

}


