package org.firstinspires.ftc.teamcode.auto.endConditions;

import org.firstinspires.ftc.teamcode.auto.structure.IEndCondition;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SensorSystem;

public class ObstacleDetect implements IEndCondition {

    private SensorSystem sensorSystem;
    private double distance;

    public ObstacleDetect(SensorSystem sensorSystem, double distance) {
        this.sensorSystem = sensorSystem;
        this.distance = distance;
    }

    @Override
    public boolean isTrue() {
        return sensorSystem.obstacleInFront();
    }

    @Override
    public void start() { }
    @Override
    public void stop() { }
}
