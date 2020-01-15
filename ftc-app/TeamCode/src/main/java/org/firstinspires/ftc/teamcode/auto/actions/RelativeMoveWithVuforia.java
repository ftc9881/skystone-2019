package org.firstinspires.ftc.teamcode.auto.actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.AutoOpConfiguration;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class RelativeMoveWithVuforia extends Action {

    private static final double CLICKS_PER_INCH = 50.0;

    private PIDController anglePidController;
    private PIDController vuforiaXPidController;

    private Robot robot;
    private Vuforia vuforia;

    private int clicksError;
    private int actualClicks;
    private int decelerateClicks;
    private int accelerateClicks;
    private double basePower;
    private double targetClicks;
    private double distance;
    private Angle moveAngle;
    private Angle targetAngle;
    private double powerFactor;
    private Pose drivePose;

    private double vuforiaTargetY;

    public RelativeMoveWithVuforia(Command command) {
        this.robot = Robot.getInstance();
        vuforia = Vuforia.getInstance();

        AutoOpConfiguration config = AutoOpConfiguration.getInstance();
        clicksError = config.properties.getInt("move clicks error", 100);
        basePower = config.properties.getDouble("move base power", 0.3);

        moveAngle = command.getAngle("move angle", 0);
        targetAngle = command.getAngle("target angle", 0);
        distance = command.getDouble("distance", 5.0);
        powerFactor = command.getDouble("power", 0.5);
        accelerateClicks = command.getInt("ramp up", 0);
        decelerateClicks = command.getInt("ramp down", 0);

        double vuforiaTargetX = command.getDouble("vuforia x", 0);
        vuforiaTargetY = command.getDouble("vuforia y", 0);
        anglePidController = new PIDController(config.properties, "move", targetAngle.getRadians());
        vuforiaXPidController = new PIDController(config.properties, "vuforia", vuforiaTargetX);
    }

    public RelativeMoveWithVuforia(Command command, VisionSystem.SkystonePosition skystonePosition) {
        this(command);
        distance = command.getDouble("distance " + skystonePosition.key, distance);
        vuforiaTargetY = command.getDouble("vuforia y " + skystonePosition.key, vuforiaTargetY);
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
            return vuforia.getPose().y < vuforiaTargetY;
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

        correctedDrivePose.r = anglePidController.getCorrectedOutput(actualHeading.getRadians());
        if (vuforiaFoundSomething()) {
            double vuforiaActualX = vuforia.getPose().x;
            double vuforiaCorrectX = vuforiaXPidController.getCorrectedOutput(vuforiaActualX);
            correctedDrivePose.y += vuforiaCorrectX;
            AutoRunner.log("VuforiaCorrectX", vuforiaCorrectX);
        }

        double rampFactor = calculateRampFactor();
        correctedDrivePose.x *= rampFactor;
        correctedDrivePose.y *= rampFactor;

        robot.driveTrain.drive(correctedDrivePose, powerFactor);

        AutoRunner.log("ActualXPower", correctedDrivePose.x);
        AutoRunner.log("ActualYPower", correctedDrivePose.y);
        AutoRunner.log("RampFactor", rampFactor);
        AutoRunner.log("pidRPower", correctedDrivePose.r);
        AutoRunner.log("ActualHeadingAngle", actualHeading.getDegrees());
        AutoRunner.log("VuforiaPose", vuforia.getPose());
    }

    private double calculateRampFactor() {
        double rampValue = 1.0;
        if (actualClicks > (targetClicks - decelerateClicks)) {
            rampValue = 1.0 - (actualClicks - (targetClicks-decelerateClicks)) / (double) decelerateClicks;
        } else if (actualClicks < accelerateClicks) {
            rampValue = Math.max(Math.sqrt(actualClicks/(double)accelerateClicks), 0.1);
        }
        return Range.clip(rampValue, basePower, 1.0);
    }

    @Override
    protected void onEndRun() {
        robot.driveTrain.stop();
    }

}
