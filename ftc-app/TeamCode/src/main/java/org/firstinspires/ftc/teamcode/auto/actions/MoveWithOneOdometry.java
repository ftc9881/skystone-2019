package org.firstinspires.ftc.teamcode.auto.actions;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.AutoOpConfiguration;
import org.firstinspires.ftc.teamcode.hardware.motor.OdometryWheel;
import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.math.PIDController;

public class MoveWithOneOdometry extends Action {

    private PIDController anglePidController;

    private Robot robot;
    private OdometryWheel odometryWheel;
    private double basePower;
    private Angle moveAngle;
    private double powerFactor;
    private Pose drivePose;
    private Pose correctedDrivePose;
    private boolean useTargetAngle;
    private double travelledInches;
    private double initialInches;

    private int decelerateDistance;
    private int accelerateDistance;
    private double targetDistance;

    public MoveWithOneOdometry(Command command) {
        tag = "MoveWithOneOdometry";
        robot = Robot.getInstance();
        odometryWheel = BatMobile.getInstance().odometryY;
        initialInches = odometryWheel.getInches();

        moveAngle = command.getAngle("move angle", 0);
        targetDistance = command.getDouble("inches", 5.0);
        powerFactor = command.getDouble("power", 0.5);
        accelerateDistance = command.getInt("ramp up", 0);
        decelerateDistance = command.getInt("ramp down", 0);
        useTargetAngle = command.getBoolean("use target angle", true);
        basePower = command.getDouble("base power", 0.3);

        Angle targetAngle = command.getAngle("target angle", 0);
        AutoOpConfiguration config = AutoOpConfiguration.getInstance();
        anglePidController = new PIDController(config.properties, "move angle", targetAngle.getRadians());
    }

    public MoveWithOneOdometry(Command command, VisionSystem.SkystonePosition skystonePosition) {
        this(command);
        targetDistance = command.getDouble("inches " + skystonePosition.key, targetDistance);
    }

    @Override
    protected void onRun() {
        drivePose = new Pose(0, 0, 0);
        drivePose.x = Math.sin(moveAngle.getRadians());
        drivePose.y = Math.cos(moveAngle.getRadians());
        correctedDrivePose = new Pose(drivePose);

        AutoRunner.log("TargetDistance", targetDistance);
        AutoRunner.log("DrivePowerX", drivePose.x);
        AutoRunner.log("DrivePowerY", drivePose.y);
    }

    @Override
    protected boolean runIsComplete() {
        return reachedTargetClicks();
    }

    protected boolean reachedTargetClicks() {
        return Math.abs(travelledInches) >= targetDistance;
    }

    @Override
    protected void insideRun() {
        correctedDrivePose = new Pose(drivePose);
        travelledInches = odometryWheel.getInches() - initialInches;

        Angle heading = robot.imu.getHeading();
        if (useTargetAngle) {
            correctedDrivePose.r = anglePidController.getCorrectedOutput(heading.getRadians());
        }

        double rampFactor = calculateRampFactor();
        correctedDrivePose.x *= rampFactor;
        correctedDrivePose.y *= rampFactor;

        robot.driveTrain.drive(correctedDrivePose, powerFactor);

        AutoRunner.log("DriveX", correctedDrivePose.x);
        AutoRunner.log("DriveY", correctedDrivePose.y);
        AutoRunner.log("DriveR", correctedDrivePose.r);
        AutoRunner.log("Ramp", rampFactor);
        AutoRunner.log("Heading", heading.getDegrees());
        AutoRunner.log("TravelledInches", travelledInches);
    }

    protected double calculateRampFactor() {
        double rampValue = 1.0;
        if (decelerateDistance > 0 && travelledInches > (targetDistance - decelerateDistance)) {
            rampValue = 1.0 - (travelledInches - (targetDistance - decelerateDistance)) / (double) decelerateDistance;
        } else if (accelerateDistance > 0 && travelledInches < accelerateDistance) {
            rampValue = Math.max(Math.sqrt(travelledInches /(double) accelerateDistance), 0.1);
        }
        return Range.clip(rampValue, basePower, 1.0);
    }

    @Override
    protected void onEndRun() {
        robot.driveTrain.stop();
    }

}
