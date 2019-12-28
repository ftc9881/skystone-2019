package org.firstinspires.ftc.teamcode.auto.actions;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.structure.AutoOpConfiguration;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;

public class AbsoluteTurn extends Action {

    Robot robot;
    private PIDController pidController;
    private double targetAngle;
    private double currentRadians;
    private double powerFactor;
    private double basePower;
    private Angle errorRange;

    public AbsoluteTurn(Angle angleToTurn, double powerFactor) {
        this.robot = Robot.getInstance();
        this.targetAngle = angleToTurn.getRadians();
        this.powerFactor = powerFactor;

        AutoOpConfiguration config = AutoOpConfiguration.getInstance();
        errorRange = config.properties.getAngle("turn error", 10, AngleUnit.DEGREES);
        basePower = config.properties.getDouble("turn base power", 0.1);
        double kP = config.properties.getDouble("turn kp", 0);
        double kI = config.properties.getDouble("turn ki", 0);
        double kD = config.properties.getDouble("turn kd", 0);
        pidController = new PIDController(kP ,kI ,kD, targetAngle);
    }

    @Override
    protected void onRun() {
        currentRadians = robot.getImuHeading().getRadians();
        AutoRunner.log("AngleToTurn", targetAngle);
        AutoRunner.log("CurrentAngle", currentRadians);
    }

    @Override
    protected boolean runIsComplete() {
        double errorRadians = Math.abs(currentRadians - targetAngle);
        return errorRadians < errorRange.getRadians();
    }

    @Override
    protected void insideRun() {
        double power = getCorrectedPower() * powerFactor;
        robot.driveTrain.drive(new Pose(0, 0, power));
        AutoRunner.log("TurnPower", power);
        Angle currentAngle = new Angle(currentRadians, AngleUnit.RADIANS);
        AutoRunner.log("Angle", currentAngle.getDegrees());
    }

    private double getCorrectedPower() {
        currentRadians = robot.getImuHeading().getRadians();
        double pidCorrectedPower = pidController.getCorrectedOutput(currentRadians);
        int sign = pidCorrectedPower > 0 ? 1 : -1;
        return Range.clip(Math.abs(pidCorrectedPower), basePower, 1.0) * sign;
    }

    @Override
    protected void onEndRun() {
        robot.driveTrain.stop();
    }
}
