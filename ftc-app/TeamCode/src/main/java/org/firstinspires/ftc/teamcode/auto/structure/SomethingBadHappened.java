package org.firstinspires.ftc.teamcode.auto.structure;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;

public class SomethingBadHappened extends RuntimeException {

    public SomethingBadHappened(String message) {
        AutoRunner.log("SomethingBadHappened", message);
    }

}
