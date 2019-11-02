package org.firstinspires.ftc.teamcode.auto.Actions;

import org.firstinspires.ftc.teamcode.Robot;

public class ActionFactory {
    Robot robot;

    public ActionFactory(Robot robot) {
        this.robot = robot;
    }

    public RelativeMove relativeMove(double inchesToMove, RelativeMove.Direction direction) {
        return new RelativeMove(robot, inchesToMove, direction);
    }


}
