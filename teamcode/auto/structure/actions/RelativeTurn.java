package org.firstinspires.ftc.teamcode.auto.structure.actions;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.structure.Action;

import static android.os.SystemClock.sleep;

public class RelativeTurn extends Action {

    private static final double DEGREES_ERROR_RANGE = 5;

    double angleToTurn;

    Robot robot;

    public RelativeTurn(Robot robot, double angleToTurn, double power) {
        this.robot = robot;
        this.angleToTurn = angleToTurn;
    }

    public void run() {
        //reference to robot encoders
        double initialAngle = robot.imu.getAngularOrientation().firstAngle;

        double currentAngle = robot.imu.getAngularOrientation().firstAngle;
        while (!isStopped && Math.abs(currentAngle - angleToTurn) > DEGREES_ERROR_RANGE) {
            currentAngle = robot.imu.getAngularOrientation().firstAngle;
            double error = (angleToTurn+initialAngle)-currentAngle;
            robot.drive(0, 0, (error*0.2)+(error>0?0.2:-0.2));
            sleep(40);
        }
        robot.drive(0, 0, 0);
    }
}
