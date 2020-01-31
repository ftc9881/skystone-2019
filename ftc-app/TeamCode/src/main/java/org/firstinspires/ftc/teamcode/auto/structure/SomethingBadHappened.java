package org.firstinspires.ftc.teamcode.auto.structure;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;

public class SomethingBadHappened extends InterruptedException {

    public SomethingBadHappened(String message) {
        AutoRunner.log("SomethingBadHappened", message);
        RobotLog.ee("SomethingBadHappened", message);
    }

}
