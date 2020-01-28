package org.firstinspires.ftc.teamcode.auto.actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.GeneralMath.Conditional;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class RelativeMoveWithVuforia extends RelativeMove {

    private PIDController vuforiaXPidController;
    private PIDController vuforiaYPidController;
    private Vuforia vuforia;
    private double vuforiaTargetY;
    private Conditional vuforiaYStopConditional;
    private double vuforiaBasePower;
    private double vuforiaCloseThreshold;

    public RelativeMoveWithVuforia(Command command) {
        super(command);

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
        Angle actualHeading = robot.getImuHeading();
        Pose vuforiaPose = vuforia.getPose();

        correctedDrivePose.r = anglePidController.getCorrectedOutput(actualHeading.getRadians());
        if (vuforiaFoundSomething()) {
            double vuforiaCorrectX = vuforiaXPidController.getCorrectedOutput(vuforiaPose.x);
            correctedDrivePose.x += vuforiaCorrectX;
            AutoRunner.log("VuforiaCorrectX", vuforiaCorrectX);

            double vuforiaCorrectY = -vuforiaYPidController.getCorrectedOutput(vuforiaPose.y);
            correctedDrivePose.y = clipPower(vuforiaCorrectY, vuforiaBasePower);
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

    private boolean vuforiaFoundSomething() {
        Pose pose = vuforia.getPose();
        return pose.x + pose.y + pose.r != 0;
    }

    private double clipPower(double power, double basePower) {
        int sign = power < 0 ? -1 : 1;
        double absPower = Math.abs(power);
        return sign * Range.clip(absPower, basePower, 1.0);
    }

    @Override
    protected void onEndRun() {
        AutoRunner.log("MoveVuforia", "onEndRun");
        robot.driveTrain.stop();
        vuforia.stopLook();
    }

}
