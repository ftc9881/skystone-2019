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
import org.firstinspires.ftc.teamcode.teleop.utility.Command;

public class BadTurn extends Action {

    public enum Direction { CLOCKWISE, COUNTER_CLOCKWISE, CLOSEST }

    private Robot robot;
    private PIDController pidController;
    private Angle targetAngle;
    private double currentRadians;
    private double powerFactor;
    private double basePower;
    private Angle errorRange;


    public BadTurn(Command command) {
        this.robot = Robot.getInstance();
        targetAngle = command.getAngle("angle", 0);
        powerFactor = command.getDouble("power", 1.0);
        errorRange = command.getAngle("error", 10, AngleUnit.DEGREES);
        basePower = command.getDouble("base power", 0.1);
        pidController = new PIDController(command, targetAngle.getRadians());
    }


    @Override
    protected void onRun() {
        currentRadians = robot.getImuHeading().getRadians();
        AutoRunner.log("AngleToTurn", targetAngle.getDegrees());
        AutoRunner.log("CurrentAngle", currentRadians);
    }

    @Override
    protected boolean runIsComplete() {
        double errorRadians = Math.abs(currentRadians - targetAngle.getRadians());
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
