package org.firstinspires.ftc.teamcode.auto.actions;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;

public class RelativeMove extends Action {

    public enum Direction {
        FRONT,
        LEFT,
        RIGHT,
        BACK
    }

    private static final double CLICKS_PER_INCH = 50.0;
    private static final double CLICKS_ERROR_RANGE = 20.0;

    Robot robot;
    private Direction direction;
    private double inchesToMove;
    private double[] targetClicks;


    public RelativeMove (Robot robot, double inchesToMove, Direction direction) {
        this.robot = robot;
        this.inchesToMove = inchesToMove;
        this.direction = direction;
    }


    @Override
    protected void onRun() {
        targetClicks = new double[4];
        for (int i=0; i < targetClicks.length; i++)
            targetClicks[i]= inchesToMove*CLICKS_PER_INCH;

        switch (direction) {
            case FRONT:
                robot.drive(1, 0, 0);
                break;
            case LEFT:
                robot.drive(0, -1, 0);
                targetClicks[0]*=-1;
                targetClicks[3]*=-1;
                break;
            case RIGHT:
                robot.drive(0, 1, 0);
                targetClicks[1]*=-1;
                targetClicks[2]*=-1;
                break;
            case BACK:
                robot.drive(-1, 0, 0);
                for (int i=0; i < targetClicks.length; i++)
                    targetClicks[i] *= -1;
                break;
        }
    }

    @Override
    protected boolean runIsComplete() {
        AutoRunner.log("LF Current Position", robot.lf.getCurrentPosition());
        AutoRunner.log("LF Target Clicks", targetClicks[0]);
        AutoRunner.log("RF Current Position", robot.rf.getCurrentPosition());
        AutoRunner.log("RF Target Clicks", targetClicks[1]);
        return Math.abs(robot.lf.getCurrentPosition() - targetClicks[0]) < CLICKS_ERROR_RANGE &&
            Math.abs(robot.rf.getCurrentPosition() - targetClicks[1]) < CLICKS_ERROR_RANGE &&
            Math.abs(robot.lb.getCurrentPosition() - targetClicks[2]) < CLICKS_ERROR_RANGE &&
            Math.abs(robot.rb.getCurrentPosition() - targetClicks[3]) < CLICKS_ERROR_RANGE;
    }

    @Override
    protected void insideRun() {
        // TODO: pid?
    }

    @Override
    protected void onEndRun() {
        robot.stop();
    }

}
