package org.firstinspires.ftc.teamcode.auto.actions;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.vision.OpenCV;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.math.GeneralMath.Conditional;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;

@Deprecated
public class MoveWithOpenCV extends MoveWithClicks {

    private PIDController vumarkXPID;
    private PIDController vumarkYPID;
    private OpenCV openCV;
    private double vumarkTargetY;
    private Conditional vumarkStopConditional;
    private double vumarkCloseThreshold;
    private Pose vumarkPose;

    public MoveWithOpenCV(Command command) {
        super(command);
        tag = "RelativeMoveWithOpenCV";

        openCV = new OpenCV(config.properties, VisionSystem.CameraType.FRONT_WEBCAM);
        openCV.startLook(VisionSystem.TargetType.PERIMETER);

        vumarkCloseThreshold = command.getDouble("vumark close threshold", 4);
        vumarkStopConditional = Conditional.convertString(command.getString("vumark stop when", "close"));

        double vumarkTargetX = command.getDouble("vumark x", 0);
        vumarkXPID = new PIDController(command, "vumark x", vumarkTargetX);
        vumarkTargetY = command.getDouble("vumark y", 0);
        vumarkYPID = new PIDController(command, "vumark y", vumarkTargetY);
    }

    public MoveWithOpenCV(Command command, VisionSystem.SkystonePosition skystonePosition) {
        this(command);
        targetDistance = command.getDouble("clicks " + skystonePosition.key, targetDistance);
        vumarkTargetY = command.getDouble("vumark y " + skystonePosition.key, vumarkTargetY);
        vumarkYPID = new PIDController(command, "vumark y", vumarkTargetY);
    }

    @Override
    protected boolean runIsComplete() {
        if (shouldUseVumark()) {
            return vumarkStopConditional.evaluate(openCV.getPose().y, vumarkTargetY, vumarkCloseThreshold);
        }
        else {
          return reachedTargetClicks();
        }
    }

    @Override
    protected void insideRun() {
        Pose correctedDrivePose = new Pose(drivePose);
        Angle actualHeading = robot.imu.getHeading();
        vumarkPose = openCV.getPose();

        correctedDrivePose.r = rPid.getCorrectedOutput(actualHeading.getRadians());
        if (shouldUseVumark()) {
            correctedDrivePose.x += vumarkXPID.getCorrectedOutput(vumarkPose.x);
            correctedDrivePose.y = GeneralMath.clipPower(-vumarkYPID.getCorrectedOutput(vumarkPose.y), basePower);
        }
        else {
            double rampFactor = calculateRampFactor();
            correctedDrivePose.x *= rampFactor;
            correctedDrivePose.y *= rampFactor;
            AutoRunner.log("RampFactor", rampFactor);
        }

        robot.driveTrain.drive(correctedDrivePose, powerFactor);

        AutoRunner.log("driveX", correctedDrivePose.x);
        AutoRunner.log("driveY", correctedDrivePose.y);
        AutoRunner.log("driveR", correctedDrivePose.r);
        AutoRunner.log("heading", actualHeading.getDegrees());
        AutoRunner.log("pose", vumarkPose);
    }

    private boolean shouldUseVumark() {
        return vumarkPose != null && !vumarkPose.isAllZero();
    }

    @Override
    protected void onEndRun() {
        AutoRunner.log("MoveVuforia", "onEndRun");
        openCV.stopLook();
        robot.driveTrain.stop();
        AutoRunner.log("MoveVuforia", "Stop");
    }

}
