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

import java.util.ArrayList;
import java.util.List;

public class RelativeMoveWithVuforiaInterpolated extends RelativeMove {

    private PIDController vuforiaXPidController;
    private PIDController vuforiaYPidController;
    private Vuforia vuforia;
    private double vuforiaTargetY;
    private Conditional vuforiaYStopConditional;
    private double vuforiaBasePower;
    private double vuforiaCloseThreshold;
    private double vuforiaBadRThreshold;
    private Pose previousCorrectedDrivePose;
    private Pose vuforiaPose;

    private List<Double> vuforiaYList;
    private List<Integer> clicksList;

    public RelativeMoveWithVuforiaInterpolated(Command command) {
        super(command);
        tag = "RelativeMoveWithVuforia";

        VisionSystem.TargetType target = VisionSystem.TargetType.stringToType(command.getString("vuforia target", "PERIMETER"));
        vuforia = Vuforia.getInstance();
        vuforia.startLook(target);
        vuforiaBadRThreshold = command.getDouble("vuforia bad r", 20);

        vuforiaCloseThreshold = command.getDouble("vuforia close threshold", 4);
        String vuforiaYStopConditionString = command.getString("vuforia stop when", "close");
        vuforiaYStopConditional = Conditional.convertString(vuforiaYStopConditionString);

        vuforiaBasePower = command.getDouble("base power", 0);
        double vuforiaTargetX = command.getDouble("vuforia x", 0);
        vuforiaXPidController = new PIDController(command, "vuforia x", vuforiaTargetX);
        vuforiaTargetY = command.getDouble("vuforia y", 0);
        vuforiaYPidController = new PIDController(command, "vuforia y", vuforiaTargetY);

        previousCorrectedDrivePose = drivePose;
    }

    public RelativeMoveWithVuforiaInterpolated(Command command, VisionSystem.SkystonePosition skystonePosition) {
        this(command);
        distance = command.getDouble("distance " + skystonePosition.key, distance);
        vuforiaTargetY = command.getDouble("vuforia y " + skystonePosition.key, vuforiaTargetY);
        vuforiaYPidController = new PIDController(command, "vuforia y", vuforiaTargetY);
    }

    @Override
    protected void onRun() {
        super.onRun();
        vuforiaYList = new ArrayList<>();
        clicksList = new ArrayList<>();
    }

    @Override
    protected boolean runIsComplete() {
        if (shouldUseVuforia()) {
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
        if (shouldUseVuforia()) {
            vuforiaYList.add(vuforiaPose.y);
            clicksList.add(robot.driveTrain.rb.getCurrentPosition());
            
            double vuforiaCorrectX;
            double vuforiaCorrectY;
            if (vuforiaPose.y > 0) {
                vuforiaCorrectX = vuforiaXPidController.getCorrectedOutput(vuforiaPose.x);
                correctedDrivePose.x += vuforiaCorrectX;
                vuforiaCorrectY = -vuforiaYPidController.getCorrectedOutput(vuforiaPose.y);
                correctedDrivePose.y = GeneralMath.clipPower(vuforiaCorrectY, vuforiaBasePower);

                AutoRunner.log("VuforiaCorrectX", vuforiaCorrectX);
                AutoRunner.log("VuforiaCorrectY", vuforiaCorrectY);
            } else {
                correctedDrivePose.y = previousCorrectedDrivePose.y;
            }
        }
        else {
            double rampFactor = calculateRampFactor();
            correctedDrivePose.x *= rampFactor;
            correctedDrivePose.y *= rampFactor;
            AutoRunner.log("RampFactor", rampFactor);
        }

        previousCorrectedDrivePose = correctedDrivePose;
        robot.driveTrain.drive(correctedDrivePose, powerFactor);

        AutoRunner.log("ActualXPower", correctedDrivePose.x);
        AutoRunner.log("ActualYPower", correctedDrivePose.y);
        AutoRunner.log("pidRPower", correctedDrivePose.r);
        AutoRunner.log("ActualHeadingAngle", actualHeading.getDegrees());
        AutoRunner.log("VuforiaPose", vuforiaPose);
    }

    private boolean shouldUseVuforia() {
        return vuforiaPose != null && !vuforiaPose.isAllZero() && Math.abs(vuforiaPose.r) < vuforiaBadRThreshold;
    }

    @Override
    protected void onEndRun() {
        AutoRunner.log("MoveVuforia", "onEndRun");
        vuforia.stopLook();
        robot.driveTrain.stop();
        AutoRunner.log("MoveVuforia", "Stop");
    }

}
