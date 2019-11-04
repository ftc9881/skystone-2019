package org.firstinspires.ftc.teamcode.auto.endConditions;

import org.firstinspires.ftc.teamcode.auto.structure.IEndCondition;
import org.firstinspires.ftc.teamcode.auto.structure.IPoseChanger;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.utility.Pose;

public class VuforiaLook implements IEndCondition, IPoseChanger {

    private Vuforia vuforia;
    private Vuforia.TargetType targetType;

    public VuforiaLook(Vuforia vuforia, Vuforia.TargetType targetType) {
        this.vuforia = vuforia;
        this.targetType = targetType;
    }

    @Override
    public void start() {
        vuforia.startLook(targetType);
    }

    @Override
    public boolean isTrue() {
        return vuforia.found();
    }

    @Override
    public void stop() {
        vuforia.stopLook();
    }

    @Override
    public Pose getPose() {
        return vuforia.getPose();
    }
}
