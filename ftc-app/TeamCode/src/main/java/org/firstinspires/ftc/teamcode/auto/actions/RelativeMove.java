package org.firstinspires.ftc.teamcode.auto.actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.AutoOpConfiguration;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.math.PIDController;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class RelativeMove extends Action {

    protected static final double CLICKS_PER_INCH = 50.0;

    protected PIDController anglePidController;

    protected Robot robot;
    protected AutoOpConfiguration config;
    protected int clicksError;
    protected int actualClicks;
    protected int decelerateClicks;
    protected int accelerateClicks;
    protected double basePower;
    protected double targetClicks;
    protected double distance;
    protected Angle moveAngle;
    protected Angle targetAngle;
    protected double powerFactor;
    protected Pose drivePose;

    protected boolean useTargetAngle;

    public RelativeMove(Command command) {
        robot = Robot.getInstance();
        moveAngle = command.getAngle("move angle", 0);
        targetAngle = command.getAngle("target angle", 0);
        distance = command.getDouble("distance", 5.0);
        powerFactor = command.getDouble("power", 0.5);
        accelerateClicks = command.getInt("ramp up", 0);
        decelerateClicks = command.getInt("ramp down", 0);
        useTargetAngle = command.getBoolean("use target angle", true);

        config = AutoOpConfiguration.getInstance();
        anglePidController = new PIDController(config.properties, "move", targetAngle.getRadians());
        clicksError = config.properties.getInt("move clicks error", 100);
        basePower = config.properties.getDouble("move base power", 0.3);
    }

    public RelativeMove(Command command, VisionSystem.SkystonePosition skystonePosition) {
        this(command);
        distance = command.getDouble("distance " + skystonePosition.key, distance);
    }

    @Override
    protected void onRun() {
        robot.driveTrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetClicks = Math.abs(distance) * CLICKS_PER_INCH;
        AutoRunner.log("TargetClicks", targetClicks);

        drivePose = new Pose(0, 0, 0);
        drivePose.x = Math.cos(moveAngle.getRadians()) * (distance<0?-1:1);
        drivePose.y = Math.sin(moveAngle.getRadians()) * (distance<0?-1:1);
        AutoRunner.log("DrivePowerX", drivePose.x);
        AutoRunner.log("DrivePowerY", drivePose.y);
    }

    @Override
    protected boolean runIsComplete() {
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

    @Override
    protected void insideRun() {
        Pose correctedDrivePose = new Pose(drivePose);

        Angle actualHeading = robot.getImuHeading();
        if (useTargetAngle) {
            correctedDrivePose.r = anglePidController.getCorrectedOutput(actualHeading.getRadians());
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
    }

    protected double calculateRampFactor() {
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
