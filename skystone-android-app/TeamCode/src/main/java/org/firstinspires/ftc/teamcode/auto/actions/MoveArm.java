package org.firstinspires.ftc.teamcode.auto.actions;

import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class MoveArm extends Action {

    private Arm arm;
    private Arm.State state;

    public MoveArm (Arm.State state) {
        this.arm = Robot.getInstance().arm;
        this.state = state;
    }

    @Override
    protected void onRun() {
        arm.move(state);
    }
    @Override
    protected boolean runIsComplete() {
        return arm.moveIsDone();
    }
    @Override
    protected void insideRun() { }

    @Override
    protected void onEndRun() {
        arm.stop();
    }
}
