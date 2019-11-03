package org.firstinspires.ftc.teamcode.auto.structure.endConditions;

import org.firstinspires.ftc.teamcode.auto.structure.EndCondition;

public class Timeout implements EndCondition {
    double startTime;
    double waitTime;

    public Timeout(double milliseconds) {
        this.startTime = System.currentTimeMillis();
        this.waitTime = milliseconds;
    }

    public boolean isTrue() {
        return (System.currentTimeMillis() - startTime) < waitTime;
    }
}
