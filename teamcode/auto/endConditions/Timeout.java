package org.firstinspires.ftc.teamcode.auto.endConditions;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.IEndCondition;

public class Timeout implements IEndCondition {
    private double startTime;
    private double waitTime;

    public Timeout(double milliseconds) {
        this.waitTime = milliseconds;
    }

    @Override
    public boolean isTrue() {
        AutoRunner.log("ElapsedTime", System.currentTimeMillis() - startTime);
        return (System.currentTimeMillis() - startTime) > waitTime;
    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void stop() {

    }

}
