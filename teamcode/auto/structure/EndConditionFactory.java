package org.firstinspires.ftc.teamcode.auto.structure;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.endConditions.SkystoneRecognition;
import org.firstinspires.ftc.teamcode.auto.endConditions.Timeout;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;

public class EndConditionFactory {
    Robot robot;
    Vuforia vuforia;

    public EndConditionFactory(Robot robot) {
        this.robot = robot;
        this.vuforia= new Vuforia(robot);
    }

    public Timeout timeout(double milliseconds) {
        return new Timeout(milliseconds);
    }

    public SkystoneRecognition skystoneRecognition() {
        return new SkystoneRecognition(vuforia);
    }

}
