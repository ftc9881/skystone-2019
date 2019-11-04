package org.firstinspires.ftc.teamcode.auto.structure;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.actions.RelativeTurn;
import org.firstinspires.ftc.teamcode.utility.Odometry;
import org.firstinspires.ftc.teamcode.auto.actions.RelativeMove;
import org.firstinspires.ftc.teamcode.auto.actions.OdometryMove;
import org.firstinspires.ftc.teamcode.utility.Pose;

public class ActionFactory {
    Robot robot;
    Odometry odometry;

    public ActionFactory(Robot robot) {
        this.robot = robot;
        this.odometry = new Odometry(robot);
    }

    public OdometryMove odometryMove(Pose targetPose, double powerFactor) {
        return new OdometryMove(robot, odometry, targetPose, powerFactor);
    }

    public RelativeMove relativeMove(double inches, RelativeMove.Direction direction) {
        return new RelativeMove(robot, inches, direction);
    }

    public RelativeTurn relativeTurn(double angle, double power) {
        return new RelativeTurn(robot, angle, power);
    }

}
