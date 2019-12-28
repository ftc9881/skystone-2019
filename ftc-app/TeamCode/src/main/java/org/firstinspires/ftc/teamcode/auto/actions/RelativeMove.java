package org.firstinspires.ftc.teamcode.auto.actions;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.AutoOpConfiguration;
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

    Robot robot;
    private int clicksError;
    private int actualClicks;
    private int decelerateClicks;
    private int accelerateClicks;
    private double basePower;
    private double targetClicks;
    private double distance;
    private Angle angle;
    private double powerFactor;
    private Pose drivePose;


    public RelativeMove(double distance, Angle angle, double powerFactor) {
        this.robot = Robot.getInstance();
        this.distance = distance;
        this.angle = angle;
        this.powerFactor = powerFactor;
    }


    @Override
    protected void onRun() {
        robot.driveTrain.resetEncoders();
        targetClicks = distance * CLICKS_PER_INCH;
        AutoRunner.log("TargetClicks", targetClicks);

        drivePose = new Pose(0, 0, 0);
        drivePose.x = Math.cos(angle.getRadians());
        drivePose.y = Math.sin(angle.getRadians());
        AutoRunner.log("DrivePowerX", drivePose.x);
        AutoRunner.log("DrivePowerY", drivePose.y);

        AutoOpConfiguration config = AutoOpConfiguration.getInstance();
        double kP = config.properties.getDouble("move kp", 0);
        double kI = config.properties.getDouble("move ki", 0);
        double kD = config.properties.getDouble("move kd", 0);
        decelerateClicks = config.properties.getInt("move decelerate clicks", 500);
        accelerateClicks = config.properties.getInt("move accelerate clicks", 500);
        clicksError = config.properties.getInt("move clicks error", 100);
        basePower = config.properties.getDouble("move base power", 0.3);
        this.pidController = new PIDController(kP, kI, kD, robot.getImuHeading().getRadians());
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
        double actualValue = robot.getImuHeading().getRadians();
        drivePose.r = pidController.getCorrectedOutput(actualValue);
        AutoRunner.log("r power", drivePose.r);

        double rampFactor = calculateRampFactor();
        AutoRunner.log("ramp factor", rampFactor);
        robot.driveTrain.drive(drivePose, powerFactor * rampFactor);
    }

    private double calculateRampFactor() {
        double rampValue = 1.0;

        if (actualClicks > (targetClicks - decelerateClicks)) {
//            rampValue = Math.pow(1.0 - (actualClicks - (targetClicks-decelerateClicks)) / (double) decelerateClicks, 2);
            rampValue = 1.0 - (actualClicks - (targetClicks-decelerateClicks)) / (double) decelerateClicks;
        }
        else if (actualClicks < accelerateClicks) {
            rampValue = Math.sqrt(actualClicks/(double)accelerateClicks);
        }

        return Range.clip(rampValue, basePower, 1.0);
    }

    @Override
    protected void onEndRun() {
        robot.driveTrain.stop();
    }

}
