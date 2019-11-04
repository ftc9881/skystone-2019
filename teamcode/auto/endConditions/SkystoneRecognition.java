package org.firstinspires.ftc.teamcode.auto.endConditions;

import org.firstinspires.ftc.teamcode.auto.structure.IEndCondition;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;

public class SkystoneRecognition implements IEndCondition {
    Vuforia vuforia;

    public SkystoneRecognition (Vuforia vuforia) {
        this.vuforia = vuforia;
    }

    @Override
    public void start() {
        vuforia.startLook(Vuforia.TargetType.SKYSTONE);
    }

    @Override
    public boolean isTrue() {
        return vuforia.found();
    }

    @Override
    public void stop() {
        vuforia.stopLook();
    }
}
