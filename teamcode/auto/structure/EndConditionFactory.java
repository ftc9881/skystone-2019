package org.firstinspires.ftc.teamcode.auto.structure;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.endConditions.ObstacleDetect;
import org.firstinspires.ftc.teamcode.auto.endConditions.VuforiaLook;
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

    public VuforiaLook vuforiaLook(Vuforia.TargetType targetType) {
        return new VuforiaLook(vuforia, targetType);
    }

    public ObstacleDetect obstacleDetect(double distance) {
        return new ObstacleDetect(robot, distance);
    }

}
