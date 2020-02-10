package org.firstinspires.ftc.teamcode.auto.actions;

import org.apache.commons.math3.stat.regression.SimpleRegression;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.math.GeneralMath.Conditional;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;

public class RelativeMoveWithVuforia extends RelativeMove {

    private PIDController vuforiaXPidController;
    private PIDController vuforiaYPidController;
    private Vuforia vuforia;
    private double vuforiaTargetY;
    private Conditional vuforiaYStopConditional;
    private double vuforiaBasePower;
    private double vuforiaCloseThreshold;
    private Pose vuforiaPose;
    private SimpleRegression regression;

    public RelativeMoveWithVuforia(Command command) {
        super(command);
        tag = "RelativeMoveWithVuforia";

        regression = new SimpleRegression();

        VisionSystem.TargetType target = VisionSystem.TargetType.stringToType(command.getString("vuforia target", "PERIMETER"));
        vuforia = Vuforia.getInstance();
        vuforia.startLook(target);

        vuforiaCloseThreshold = command.getDouble("vuforia close threshold", 4);
        String vuforiaYStopConditionString = command.getString("vuforia stop when", "close");
        vuforiaYStopConditional = Conditional.convertString(vuforiaYStopConditionString);

        vuforiaBasePower = command.getDouble("base power", 0);
        double vuforiaTargetX = command.getDouble("vuforia x", 0);
        vuforiaXPidController = new PIDController(command, "vuforia x", vuforiaTargetX);
        vuforiaTargetY = command.getDouble("vuforia y", 0);
        vuforiaYPidController = new PIDController(command, "vuforia y", vuforiaTargetY);
    }

    public RelativeMoveWithVuforia(Command command, VisionSystem.SkystonePosition skystonePosition) {
        this(command);
        distance = command.getDouble("distance " + skystonePosition.key, distance);
        vuforiaTargetY = command.getDouble("vuforia y " + skystonePosition.key, vuforiaTargetY);
        vuforiaYPidController = new PIDController(command, "vuforia y", vuforiaTargetY);
    }

    @Override
    protected boolean runIsComplete() {
        if (vuforiaFoundSomething()) {
            return vuforiaYStopConditional.evaluate(vuforia.getPose().y, vuforiaTargetY, vuforiaCloseThreshold);
        }
        else {
          return reachedTargetClicks();
        }
    }

    @Override
    protected void insideRun() {
        Pose correctedDrivePose = new Pose(drivePose);
        Angle actualHeading = robot.imu.getHeading();
        vuforiaPose = vuforia.getPose();

        correctedDrivePose.r = anglePidController.getCorrectedOutput(actualHeading.getRadians());
        if (vuforiaFoundSomething()) {
            double vuforiaY = vuforiaPose.y;
            if (vuforiaIsReasonable()) {
                regression.addData(getTrackingClicks(), vuforiaPose.y);
                correctedDrivePose.x += vuforiaXPidController.getCorrectedOutput(vuforiaPose.x);
            } else {
                vuforiaY = getPredictedY();
                AutoRunner.log("PredictedY", vuforiaY);
            }
            correctedDrivePose.y = GeneralMath.clipPower(-vuforiaYPidController.getCorrectedOutput(vuforiaY), vuforiaBasePower);
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
        AutoRunner.log("VuforiaPose", vuforiaPose);
    }

    @Override
    protected void onEndRun() {
        vuforia.stopLook();
        robot.driveTrain.stop();
    }

    private double getTrackingClicks() {
        // TODO: May want to get averaged motor clicks
        return robot.driveTrain.rf.getCurrentPosition();
    }

    private boolean vuforiaFoundSomething() {
        return vuforiaPose != null && !vuforiaPose.isAllZero();
    }

    private boolean vuforiaIsReasonable() {
        return vuforiaPose.y > 0;
    }

    private double getPredictedY() {
        return regression.getSlope() * getTrackingClicks() + regression.getIntercept();
    }

}
