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
import java.util.List;

public class MoveWithClicks extends Action {

    protected PIDController rPid;

    protected Robot robot;
    protected AutoOpConfiguration config;
    protected double basePower;
    protected Angle moveAngle;
    protected Angle targetAngle;
    protected double powerFactor;
    protected Pose drivePose;
    protected Pose correctedDrivePose;
    protected boolean useTargetAngle;
    protected int currentClicks;

    protected int clicksError;
    protected int decelerateDistance;
    protected int accelerateDistance;
    protected double targetDistance;

    public MoveWithClicks(Command command) {
        tag = "RelativeMove";
        robot = Robot.getInstance();
        moveAngle = command.getAngle("move angle", 0);
        targetAngle = command.getAngle("target angle", 0);
        targetDistance = command.getDouble("clicks", 5.0);
        powerFactor = command.getDouble("power", 0.5);
        accelerateDistance = command.getInt("ramp up", 0);
        decelerateDistance = command.getInt("ramp down", 0);
        useTargetAngle = command.getBoolean("use target angle", true);

        config = AutoOpConfiguration.getInstance();
        rPid = new PIDController(config.properties, "move angle", targetAngle.getRadians());
        clicksError = command.getInt("move clicks error", 100);
        basePower = command.getDouble("base power", 0.3);
    }

    public MoveWithClicks(Command command, VisionSystem.SkystonePosition skystonePosition) {
        this(command);
        targetDistance = command.getDouble("clicks " + skystonePosition.key, targetDistance);
    }

    @Override
    protected void onRun() {
        robot.driveTrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetDistance = Math.abs(targetDistance);

        drivePose = new Pose(0, 0, 0);
        drivePose.x = Math.sin(moveAngle.getRadians());
        drivePose.y = Math.cos(moveAngle.getRadians());
        correctedDrivePose = new Pose(drivePose);

        AutoRunner.log("TargetClicks", targetDistance);
        AutoRunner.log("DrivePowerX", drivePose.x);
        AutoRunner.log("DrivePowerY", drivePose.y);
    }

    @Override
    protected boolean runIsComplete() {
        return reachedTargetClicks();
    }

    protected boolean reachedTargetClicks() {
//        currentClicks = Math.abs((int) robot.driveTrain.getAverageClicks());
        currentClicks = Math.abs(robot.driveTrain.rb.getCurrentPosition());
        return Math.abs(currentClicks - targetDistance) < clicksError;
    }

    @Override
    protected void insideRun() {
        correctedDrivePose = new Pose(drivePose);

        Angle actualHeading = robot.imu.getHeading();
        if (useTargetAngle) {
            correctedDrivePose.r = rPid.getCorrectedOutput(actualHeading.getRadians());
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
        if (decelerateDistance > 0 && currentClicks > (targetDistance - decelerateDistance)) {
            rampValue = 1.0 - (currentClicks - (targetDistance - decelerateDistance)) / (double) decelerateDistance;
        } else if (accelerateDistance > 0 && currentClicks < accelerateDistance) {
            rampValue = Math.max(Math.sqrt(currentClicks /(double) accelerateDistance), 0.1);
        }
        return Range.clip(rampValue, basePower, 1.0);
    }

    protected List<Integer> getClicksArray() {
        List<Integer> clicksArray = new ArrayList<>();
        clicksArray.add(Math.abs(robot.driveTrain.lf.getCurrentPosition()));
        clicksArray.add(Math.abs(robot.driveTrain.rf.getCurrentPosition()));
        clicksArray.add(Math.abs(robot.driveTrain.lb.getCurrentPosition()));
        clicksArray.add(Math.abs(robot.driveTrain.rb.getCurrentPosition()));
        return clicksArray;
    }

    @Override
    protected void onEndRun() {
        robot.driveTrain.stop();
    }

}
