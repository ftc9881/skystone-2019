package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;

@Autonomous(name = "Build Side", group = "TeamCode")
@Disabled
public class BuildSideAutoOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        AutoRunner auto = new AutoRunner("BuildSide", this);
        waitForStart();
        auto.run();
    }

}

