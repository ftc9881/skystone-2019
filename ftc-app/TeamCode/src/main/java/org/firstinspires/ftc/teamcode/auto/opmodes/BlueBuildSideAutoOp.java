package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;

@Autonomous(name = "Blue Build Side", group = "TeamCode")
public class BlueBuildSideAutoOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        AutoRunner auto = new AutoRunner("BlueBuildSide", this);
        waitForStart();
        auto.run();
    }

}

