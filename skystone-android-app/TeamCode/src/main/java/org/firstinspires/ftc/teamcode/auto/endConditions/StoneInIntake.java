package org.firstinspires.ftc.teamcode.auto.endConditions;

import org.firstinspires.ftc.teamcode.auto.structure.IEndCondition;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SensorSystem;

public class StoneInIntake implements IEndCondition {

    private SensorSystem sensorSystem;

    public StoneInIntake() {
        this.sensorSystem = Robot.getInstance().sensorSystem;
    }

    @Override
    public boolean isTrue() {
        return sensorSystem.stoneIsIn();
    }

    @Override
    public void start() { }
    @Override
    public void stop() { }
}
