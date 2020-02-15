package org.firstinspires.ftc.teamcode.auto.actions;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.math.GeneralMath.Conditional;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;

public class MoveCombined extends Move {

    private PIDController vuforiaXPidController;
    private PIDController vuforiaYPidController;
    private Vuforia vuforia;
    private double targetVY;
    private double targetVX;
    private Conditional conditionalVY;
    private double closeVY;
    private Conditional conditionalVX;
    private double closeVX;
    private double vuforiaBasePower;
    private Pose pose = new Pose();
    private Pose lastPose = new Pose();

    private int clicksUntilCorrectX;

    private double jumpThreshold;
    private double lastCorrectedDriveY;

    private double minimumLookTime;
    private double startTime;

    public MoveCombined(Command command) {
        super(command);
        tag = "RelativeMoveCombined";

        vuforia = Vuforia.getInstance();
        if (!vuforia.isLooking()) {
            VisionSystem.TargetType target = VisionSystem.TargetType.stringToType(command.getString("vuforia target", "PERIMETER"));
            vuforia.startLook(target);
        }

        jumpThreshold = command.getDouble("jump threshold", 1);

        clicksUntilCorrectX = command.getInt("vuforia x clicks until correct", 0);

        closeVY = command.getDouble("vuforia y close threshold", 4);
        String conditionalString = command.getString("vuforia y stop when", "close");
        conditionalVY = Conditional.convertString(conditionalString);

        closeVX = command.getDouble("vuforia x close threshold", 4);
        conditionalVX = Conditional.CLOSE;
        minimumLookTime = command.getDouble("vuforia min look time", 1000);

        vuforiaBasePower = command.getDouble("base power", 0);
        targetVX = command.getDouble("vuforia x", 0);
        vuforiaXPidController = new PIDController(command, "vuforia x", targetVX);
        targetVY = command.getDouble("vuforia y", 0);
        vuforiaYPidController = new PIDController(command, "vuforia y", targetVY);
    }

    public MoveCombined(Command command, VisionSystem.SkystonePosition skystonePosition) {
        this(command);
        clicks = command.getDouble("clicks " + skystonePosition.key, clicks);
        targetVY = command.getDouble("vuforia y " + skystonePosition.key, targetVY);
        vuforiaYPidController = new PIDController(command, "vuforia y", targetVY);
    }

    @Override
    protected void onRun() {
        super.onRun();
        startTime = System.currentTimeMillis();
        lastCorrectedDriveY = drivePose.y;
    }

    @Override
    protected boolean runIsComplete() {
        if (vuforiaFoundSomething()) {
            return conditionalVY.evaluate(pose.y, targetVY, closeVY) && conditionalVX.evaluate(pose.x, targetVX, closeVX);
        }
        else {
          return reachedTargetClicks() && getElapsedTime() > minimumLookTime;
        }
    }

    @Override
    protected void insideRun() {
        Pose correctedDrivePose = new Pose(drivePose);
        Angle actualHeading = robot.imu.getHeading();
        lastPose = pose;
        pose = vuforia.getPose();

        correctedDrivePose.r = anglePidController.getCorrectedOutput(actualHeading.getRadians());
        if (vuforiaFoundSomething()) {
            if (vuforiaIsReasonable()) {
                if (getTrackingClicks() > clicksUntilCorrectX) {
                    correctedDrivePose.x += vuforiaXPidController.getCorrectedOutput(pose.x);
                }
                correctedDrivePose.y = GeneralMath.clipPower(-vuforiaYPidController.getCorrectedOutput(pose.y), vuforiaBasePower);
                lastCorrectedDriveY = correctedDrivePose.y;
            } else {
                correctedDrivePose.y = lastCorrectedDriveY;
            }
        }
        else {
            double rampFactor = calculateRampFactor();
            correctedDrivePose.x *= rampFactor;
            correctedDrivePose.y *= rampFactor;
            AutoRunner.log("RampFactor", rampFactor);
        }

        robot.driveTrain.drive(correctedDrivePose, powerFactor);

        AutoRunner.log("ActualXPower", correctedDrivePose.x);
        AutoRunner.log("ActualYPower", correctedDrivePose.y);
        AutoRunner.log("pidRPower", correctedDrivePose.r);
        AutoRunner.log("ActualHeadingAngle", actualHeading.getDegrees());
        AutoRunner.log("VuforiaPose", pose);
        AutoRunner.log(getTrackingClicks());
    }

    @Override
    protected void onEndRun() {
        robot.driveTrain.stop();
//        vuforia.stopLook();
    }

    private double getTrackingClicks() {
        return Math.abs(robot.driveTrain.getAverageClicks());
    }

    private boolean vuforiaFoundSomething() {
        return pose != null && !pose.isAllZero();
    }

    private boolean vuforiaIsReasonable() {
//        return pose.y > 0;
        // Since getting closer, new should be less than last
        return pose.y - lastPose.y < jumpThreshold;
    }

    private double getElapsedTime() {
        return System.currentTimeMillis() - startTime;
    }


}
