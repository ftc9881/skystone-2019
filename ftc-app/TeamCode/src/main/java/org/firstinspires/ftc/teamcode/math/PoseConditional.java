package org.firstinspires.ftc.teamcode.math;

import org.firstinspires.ftc.teamcode.math.GeneralMath.Conditional;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;

public class PoseConditional {

    public Conditional conditionalX;
    public Conditional conditionalY;
    public Conditional conditionalR;
    private Pose closeThresholdPose = new Pose();
    private Pose targetPose;

    public PoseConditional(Command config, Pose targetPose) {
        this.targetPose = targetPose;
        closeThresholdPose.x = config.getDouble("x close threshold", 0.5);
        closeThresholdPose.y = config.getDouble("y close threshold", 0.5);
        closeThresholdPose.r = config.getDouble("r close threshold", 0.5);
        conditionalX = Conditional.convertString( config.getString("x stop when", "close") );
        conditionalY = Conditional.convertString( config.getString("y stop when", "close") );
        conditionalR = Conditional.convertString( config.getString("r stop when", "none") );
    }

    public boolean isMet(Pose actualPose) {
        return conditionalX.evaluate(actualPose.x, targetPose.x, closeThresholdPose.x) &&
               conditionalY.evaluate(actualPose.y, targetPose.y, closeThresholdPose.y) &&
               conditionalR.evaluate(actualPose.r, targetPose.r, closeThresholdPose.r);
    }

}
