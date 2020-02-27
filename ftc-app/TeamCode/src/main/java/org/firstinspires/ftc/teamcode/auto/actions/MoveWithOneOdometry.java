package org.firstinspires.ftc.teamcode.auto.actions;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.AutoOpConfiguration;
import org.firstinspires.ftc.teamcode.hardware.motor.OdometryWheel;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.math.PIDController;

@Deprecated
public class MoveWithOneOdometry extends Action {

    private PIDController anglePidController;

    private Robot robot;
    private OdometryWheel odometryWheel;
    private Angle moveAngle;
    private double powerFactor;
    private Pose drivePose;
    private Pose correctedDrivePose;
    private boolean useTargetAngle;
    private double travelledInches = 0;
    private double initialInches;
    private double basePower;

    private PIDController pidY;
    private GeneralMath.Conditional conditionalY;
    private double targetY;
    private double closeY;

    public MoveWithOneOdometry(Command command) {
        tag = "MoveWithOneOdometry";
        robot = Robot.getInstance();
        odometryWheel = BatMobile.getInstance().odometryY;
        initialInches = odometryWheel.getInches();

        basePower = command.getDouble("base power", 0.3);
        moveAngle = command.getAngle("move angle", 0);
        targetY = command.getDouble("inches", 5.0);
        powerFactor = command.getDouble("power", 0.5);
        useTargetAngle = command.getBoolean("use target angle", true);

        Angle targetAngle = command.getAngle("target angle", 0);
        AutoOpConfiguration config = AutoOpConfiguration.getInstance();
        anglePidController = new PIDController(config.properties, "move angle", targetAngle.getRadians());

        closeY = command.getDouble("y close threshold", 1);
        String conditionalString = command.getString("y stop when", "close");
        conditionalY = GeneralMath.Conditional.convertString(conditionalString);
        targetY = command.getDouble("target y", 0);
        pidY = new PIDController(command, "y", targetY);
    }

    public MoveWithOneOdometry(Command command, VisionSystem.SkystonePosition skystonePosition) {
        this(command);
        targetY = command.getDouble("inches " + skystonePosition.key, targetY);
        pidY = new PIDController(command, "y", targetY);
    }

    @Override
    protected void onRun() {
        drivePose = new Pose(0, 0, 0);
        drivePose.x = Math.sin(moveAngle.getRadians());
        drivePose.y = Math.cos(moveAngle.getRadians());
        correctedDrivePose = new Pose(drivePose);

        AutoRunner.log("TargetDistance", targetY);
        AutoRunner.log("DrivePowerX", drivePose.x);
        AutoRunner.log("DrivePowerY", drivePose.y);
    }

    @Override
    protected boolean runIsComplete() {
        return conditionalY.evaluate(travelledInches, targetY, closeY);
    }

    @Override
    protected void insideRun() {
        correctedDrivePose = new Pose(drivePose);
        travelledInches = Math.abs(odometryWheel.getInches() - initialInches);

        Angle heading = robot.imu.getHeading();
        if (useTargetAngle) {
            correctedDrivePose.r = anglePidController.getCorrectedOutput(heading.getRadians());
        }

        correctedDrivePose.y = GeneralMath.clipPower(-pidY.getCorrectedOutput(travelledInches), basePower);

        robot.driveTrain.drive(correctedDrivePose, powerFactor);

        AutoRunner.log("DriveX", correctedDrivePose.x);
        AutoRunner.log("DriveY", correctedDrivePose.y);
        AutoRunner.log("DriveR", correctedDrivePose.r);
        AutoRunner.log("Heading", heading.getDegrees());
        AutoRunner.log("TravelledInches", travelledInches);
    }


    @Override
    protected void onEndRun() {
        robot.driveTrain.stop();

        AutoRunner.log("TravelledInchesEND", travelledInches);
    }

}
