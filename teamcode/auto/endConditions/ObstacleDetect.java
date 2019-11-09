package org.firstinspires.ftc.teamcode.auto.endConditions;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.auto.structure.IEndCondition;

public class ObstacleDetect implements IEndCondition {
    private Robot robot;
    private double distance;

    public ObstacleDetect(Robot robot, double distance) {
        this.robot = robot;
        this.distance = distance;
    }

    @Override
    public boolean isTrue() {
        // TODO: put is something in front? function in SensorSystem
//        return robot.sonarSensor.getDistance() < distance;
        return false;
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {

    }
}
