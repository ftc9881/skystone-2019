package org.firstinspires.ftc.teamcode.auto.actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.AutoOpConfiguration;
import org.firstinspires.ftc.teamcode.auto.structure.Command;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.math.PIDController;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class RelativeMove extends Action {

    private static final double CLICKS_PER_INCH = 50.0;

    private PIDController pidController;

    private Robot robot;
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

    public RelativeMove(Command command) {
        this.robot = Robot.getInstance();
        moveAngle = command.getAngle("move angle", 0);
        targetAngle = command.getAngle("target angle", 0);
        distance = command.getDouble("distance", 5.0);
        powerFactor = command.getDouble("power", 0.5);
        accelerateClicks = command.getInt("ramp up", 0);
        decelerateClicks = command.getInt("ramp down", 0);
    }

    public RelativeMove(double distance, Angle moveAngle, Angle targetAngle, double powerFactor, int accelerateClicks, int decelerateClicks) {
        this.robot = Robot.getInstance();
        this.distance = distance;
        this.moveAngle = moveAngle;
        this.targetAngle = targetAngle;
        this.powerFactor = powerFactor;
        this.accelerateClicks = accelerateClicks;
        this.decelerateClicks = decelerateClicks;
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

        AutoOpConfiguration config = AutoOpConfiguration.getInstance();
        double kP = config.properties.getDouble("move kp", 0);
        double kI = config.properties.getDouble("move ki", 0);
        double kD = config.properties.getDouble("move kd", 0);
        clicksError = config.properties.getInt("move clicks error", 100);
        basePower = config.properties.getDouble("move base power", 0.3);
        this.pidController = new PIDController(kP, kI, kD, targetAngle.getRadians());
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

        AutoRunner.log("actual clicks", actualClicks);

        return reachedTarget;
    }

    @Override
    protected void insideRun() {
        Pose correctedDrivePose = new Pose(drivePose);
        Angle actualHeading = robot.getImuHeading();
        correctedDrivePose.r = pidController.getCorrectedOutput(actualHeading.getRadians());

        double rampFactor = calculateRampFactor();
        correctedDrivePose.x *= rampFactor;
        correctedDrivePose.y *= rampFactor;

        robot.driveTrain.drive(correctedDrivePose, powerFactor);

        AutoRunner.log("ActualXPower", correctedDrivePose.x);
        AutoRunner.log("ActualYPower", correctedDrivePose.y);

        AutoRunner.log("RampFactor", rampFactor);
        AutoRunner.log("pidRPower", correctedDrivePose.r);
        AutoRunner.log("Angle", actualHeading.getDegrees());
    }

    private double calculateRampFactor() {
        double rampValue = 1.0;

        if (actualClicks > (targetClicks - decelerateClicks)) {
//            rampValue = Math.pow(1.0 - (actualClicks - (targetClicks-decelerateClicks)) / (double) decelerateClicks, 2);
            rampValue = 1.0 - (actualClicks - (targetClicks-decelerateClicks)) / (double) decelerateClicks;
        }
        else if (actualClicks < accelerateClicks) {
            rampValue = Math.max(Math.sqrt(actualClicks/(double)accelerateClicks), 0.1);
//            rampValue = (actualClicks/(double)accelerateClicks);
        }

        return Range.clip(rampValue, basePower, 1.0);
    }

    @Override
    protected void onEndRun() {
        robot.driveTrain.stop();
    }

}
