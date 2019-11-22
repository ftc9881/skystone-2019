package org.firstinspires.ftc.teamcode.auto.endConditions;

import org.firstinspires.ftc.teamcode.auto.structure.IEndCondition;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class VuforiaLook implements IEndCondition {

    private Vuforia vuforia;
    private Vuforia.TargetType targetType;

    public VuforiaLook(Vuforia.TargetType targetType) {
        this.vuforia = Robot.getInstance().vuforia;
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

    public Pose getPose() {
        return vuforia.getPose();
    }
}
