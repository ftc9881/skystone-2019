package org.firstinspires.ftc.teamcode.auto.structure.actions;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.structure.Action;

import static android.os.SystemClock.sleep;

public class RelativeMove extends Action {

    private static final double CLICKS_PER_INCH = 200.0;
    private static final double CLICKS_ERROR_RANGE = 20.0;

    public enum Direction {
        FRONT,
        LEFT,
        RIGHT,
        BACK
    }

    Direction direction;
    double inchesToMove;

    Robot robot;

    public RelativeMove (Robot robot, double inchesToMove, Direction direction) {
        this.robot = robot;
        this.inchesToMove = inchesToMove;
        this.direction = direction;
    }

    public void run() {
        //reference to robot encoders
        double[] targetClicks = new double[4];
        for (int i=0; i < targetClicks.length; i++)
            targetClicks[i]= inchesToMove*CLICKS_PER_INCH;

        //TODO: reset encoders to 0?

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

        while (!isStopped && (
                Math.abs(robot.lf.getCurrentPosition() - targetClicks[0]) > CLICKS_ERROR_RANGE &&
                Math.abs(robot.rf.getCurrentPosition() - targetClicks[1]) > CLICKS_ERROR_RANGE &&
                Math.abs(robot.lb.getCurrentPosition() - targetClicks[2]) > CLICKS_ERROR_RANGE &&
                Math.abs(robot.rb.getCurrentPosition() - targetClicks[3]) > CLICKS_ERROR_RANGE
        )) {
            sleep(40);
        }
        robot.drive(0, 0, 0);
    }
}
