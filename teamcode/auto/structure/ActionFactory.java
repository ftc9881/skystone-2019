package org.firstinspires.ftc.teamcode.auto.structure;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.auto.structure.actions.RelativeMove;
import org.firstinspires.ftc.teamcode.auto.structure.actions.OdometryMove;

public class ActionFactory {
    Robot robot;
    Odometry odometry;

    public ActionFactory(Robot robot) {
        this.robot = robot;
        this.odometry = new Odometry(robot);
    }

    public RelativeMove relativeMove(double inchesToMove, RelativeMove.Direction direction) {
        return new RelativeMove(robot, inchesToMove, direction);
    }

    public OdometryMove odometryMove(double targetX, double targetY, double targetR, double powerFactor) {
        return new OdometryMove(robot, odometry, targetX, targetY, targetR, powerFactor);
    }

}
