package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;

@Autonomous
//@Disabled
public class Blue3Stone extends LinearOpMode {

    @Override
    public void runOpMode() {
        AutoRunner auto = new AutoRunner(AutoRunner.Side.BLUE, "RedStoneSide", this);
        waitForStart();
        auto.run();
    }

}

