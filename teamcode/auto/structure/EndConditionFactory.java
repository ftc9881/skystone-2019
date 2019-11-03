package org.firstinspires.ftc.teamcode.auto.structure;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.structure.actions.RelativeMove;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;

public class EndConditionFactory {
    Robot robot;
    Vuforia vuforia;

    public EndConditionFactory(Robot robot) {
        this.robot = robot;
        this.vuforia= new Vuforia(robot);
    }

}
