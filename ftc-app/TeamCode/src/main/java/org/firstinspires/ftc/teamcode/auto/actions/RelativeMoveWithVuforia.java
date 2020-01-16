package org.firstinspires.ftc.teamcode.auto.actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class RelativeMoveWithVuforia extends RelativeMove {

    public enum VuforiaStopCondition {
        GREATER, LESS, CLOSE;

        static public double closeThreshold = 1;

        static public VuforiaStopCondition convertString(String string) {
            switch (string.toUpperCase()) {
                case "GREATER":
                    return GREATER;
                case "LESS":
                    return LESS;
                default:
                    return CLOSE;
            }
        }

        public boolean evaluate(double actual, double target) {
            switch (this) {
                case GREATER:
                    return actual >= target;
                case LESS:
                    return actual <= target;
                case CLOSE:
                    return Math.abs(actual - target) < closeThreshold;
            }
            return false;
        }
    }

    private PIDController vuforiaXPidController;
    private PIDController vuforiaYPidController;
    private Vuforia vuforia;
    private double vuforiaTargetY;
    private VuforiaStopCondition vuforiaYStopCondition;
    private double vuforiaBasePower;

    public RelativeMoveWithVuforia(Command command) {
        super(command);
        vuforia = Vuforia.getInstance();
        vuforia.startLook(VisionSystem.TargetType.KEEP_RUNNING);

        VuforiaStopCondition.closeThreshold = command.getDouble("vuforia close threshold", 4);
        String vuforiaYStopConditionString = command.getString("vuforia stop when", "close");
        vuforiaYStopCondition = VuforiaStopCondition.convertString(vuforiaYStopConditionString);

        vuforiaBasePower = command.getDouble("base power", 0);
        double vuforiaTargetX = command.getDouble("vuforia x", 0);
        vuforiaXPidController = new PIDController(command, "vuforia x", vuforiaTargetX);
        vuforiaTargetY = command.getDouble("vuforia y", 0);
        vuforiaYPidController = new PIDController(config.properties, "vuforia y", vuforiaTargetY);
    }

    public RelativeMoveWithVuforia(Command command, VisionSystem.SkystonePosition skystonePosition) {
        this(command);
        distance = command.getDouble("distance " + skystonePosition.key, distance);
        vuforiaTargetY = command.getDouble("vuforia y " + skystonePosition.key, vuforiaTargetY);
        vuforiaYPidController = new PIDController(config.properties, "vuforia y", vuforiaTargetY);
    }


    @Override
    protected void onRun() {
        robot.driveTrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetClicks = Math.abs(distance) * CLICKS_PER_INCH;

        drivePose = new Pose(0, 0, 0);
        int direction = distance > 0 ? 1 : -1;
        drivePose.x = Math.cos(moveAngle.getRadians()) * direction;
        drivePose.y = Math.sin(moveAngle.getRadians()) * direction;

        AutoRunner.log("TargetClicks", targetClicks);
        AutoRunner.log("VuforiaTargetY", vuforiaTargetY);
        AutoRunner.log("DrivePowerX", drivePose.x);
        AutoRunner.log("DrivePowerY", drivePose.y);
    }

    private boolean vuforiaFoundSomething() {
        Pose pose = vuforia.getPose();
        return pose.x + pose.y + pose.r != 0;
    }

    @Override
    protected boolean runIsComplete() {
        if (vuforiaFoundSomething()) {
            return vuforiaYStopCondition.evaluate(vuforia.getPose().y, vuforiaTargetY);
        }
        else {
            List<Integer> clicksArray = new ArrayList<>();
            clicksArray.add(Math.abs(robot.driveTrain.lf.getCurrentPosition()));
            clicksArray.add(Math.abs(robot.driveTrain.lb.getCurrentPosition()));
            clicksArray.add(Math.abs(robot.driveTrain.rf.getCurrentPosition()));
            clicksArray.add(Math.abs(robot.driveTrain.rb.getCurrentPosition()));
            actualClicks = Collections.max(clicksArray);
            boolean reachedTarget = Math.abs(actualClicks - targetClicks) < clicksError;

            AutoRunner.log("ActualClicks", actualClicks);
            return reachedTarget;
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
            correctedDrivePose.y += vuforiaCorrectX;
            AutoRunner.log("VuforiaCorrectX", vuforiaCorrectX);

            double vuforiaCorrectY = -vuforiaYPidController.getCorrectedOutput(vuforiaPose.y);
            correctedDrivePose.x = clipPower(vuforiaCorrectY, vuforiaBasePower);
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

    protected double clipPower(double power, double basePower) {

        int sign = power < 0 ? -1 : 1;
        double absPower = Math.abs(power);
        return sign * Range.clip(absPower, basePower, 1.0);
    }

    @Override
    protected void onEndRun() {
        robot.driveTrain.stop();
        vuforia.stopLook();
    }

}
