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

public class RelativeMoveWithVuforia extends RelativeMove {

    private PIDController vuforiaXPidController;
    private PIDController vuforiaYPidController;
    private Vuforia vuforia;
    private double vuforiaTargetY;
    private Conditional vuforiaYStopConditional;
    private double vuforiaBasePower;
    private double vuforiaCloseThreshold;
    private double vuforiaBadRThreshold;

    public RelativeMoveWithVuforia(Command command) {
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
    }

    public RelativeMoveWithVuforia(Command command, VisionSystem.SkystonePosition skystonePosition) {
        this(command);
        distance = command.getDouble("distance " + skystonePosition.key, distance);
        vuforiaTargetY = command.getDouble("vuforia y " + skystonePosition.key, vuforiaTargetY);
        vuforiaYPidController = new PIDController(command, "vuforia y", vuforiaTargetY);
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
        Pose vuforiaPose = vuforia.getPose();

        correctedDrivePose.r = anglePidController.getCorrectedOutput(actualHeading.getRadians());
        if (shouldUseVuforia()) {
            double vuforiaCorrectX = vuforiaXPidController.getCorrectedOutput(vuforiaPose.x);
            correctedDrivePose.x += vuforiaCorrectX;
            AutoRunner.log("VuforiaCorrectX", vuforiaCorrectX);

            double vuforiaCorrectY = vuforiaYPidController.getCorrectedOutput(vuforiaPose.y);
            correctedDrivePose.y *= GeneralMath.clipPower(vuforiaCorrectY, vuforiaBasePower);
            AutoRunner.log("VuforiaCorrectY", vuforiaCorrectY);
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

    private boolean shouldUseVuforia() {
        return vuforiaFoundSomething() && Math.abs(vuforia.getPose().r) < vuforiaBadRThreshold;
    }

    private boolean vuforiaFoundSomething() {
        Pose pose = vuforia.getPose();
        return pose.x + pose.y + pose.r != 0;
    }

    @Override
    protected void onEndRun() {
        AutoRunner.log("MoveVuforia", "onEndRun");
        vuforia.stopLook();
        robot.driveTrain.stop();
        AutoRunner.log("MoveVuforia", "Stop");
    }

}
