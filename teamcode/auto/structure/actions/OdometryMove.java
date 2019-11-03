package org.firstinspires.ftc.teamcode.auto.structure.actions;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Pose;

import static android.os.SystemClock.sleep;

public class OdometryMove extends Action{

    Robot robot;
    Odometry odometry;

    double targetX, targetY, targetR, power;

    //needs more stuff
    public OdometryMove (Robot robot, Odometry odometry, double targetX, double targetY, double targetR, double power) {
        this.odometry = odometry;
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetR = targetR;
        this.robot = robot;
    }

    public void run() {
        //reference to robot encoders
        Pose currentPose = odometry.getPose();
//        Pose pastPose;
        //TODO: reset encoders to 0?
        Pose targetPose = new Pose(targetX, targetY, targetR);

        while (!isStopped && (currentPose.distanceTo(targetPose) > 1)
            || Math.abs(currentPose.r - targetPose.r) > 0.3
        ) {

            currentPose = odometry.getPose();

            Pose errorPose = targetPose.subtract(currentPose);
            Pose drivePose = new Pose(0, 0, 0);

            drivePose.r = Range.clip(targetPose.r - currentPose.r, -1, 1);

            double absAngleDir = Math.atan(errorPose.y / errorPose.x);
            if (errorPose.x < 0) absAngleDir += Math.PI;

            double relAngleError = absAngleDir + currentPose.r;

            double XYMagnitude = Math.sqrt(Math.pow(errorPose.x, 2) + Math.pow(errorPose.y, 2));

            drivePose.x = Range.clip(-XYMagnitude * Math.cos(relAngleError), -1, 1);
            drivePose.y = Range.clip(XYMagnitude * Math.sin(relAngleError), -1, 1);

            // only proportional correction
            robot.drive(drivePose, power);
//            opMode.telemetry.addData("CurrentPose", currentPose.toString());
            robot.log(drivePose.toString());

//          if (obstacleDetection) {
//               while (obstacle.stillThere()) {
//                  sleep(50);
//              }
//          }

//            pastPose = currentPose;
        }
        robot.drive(0, 0, 0);
        robot.log("Done with move");
    }
}
