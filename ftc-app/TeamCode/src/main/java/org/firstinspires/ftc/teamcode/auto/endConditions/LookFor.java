package org.firstinspires.ftc.teamcode.auto.endConditions;

import org.firstinspires.ftc.teamcode.auto.structure.IEndCondition;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class LookFor implements IEndCondition {

    private VisionSystem visionSystem;
    private VisionSystem.TargetType targetType;

    public LookFor(VisionSystem.TargetType targetType) {
        this.visionSystem = Robot.getInstance().visionSystem;
        this.targetType = targetType;
    }

    @Override
    public void start() {
        visionSystem.startLook(targetType);
    }

    @Override
    public boolean isTrue() {
        return visionSystem.found();
    }

    @Override
    public void stop() {
        visionSystem.stopLook();
    }

}
