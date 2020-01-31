package org.firstinspires.ftc.teamcode.auto.actions;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;

public class Turn extends Action {

    public enum Direction {
        CLOCKWISE, COUNTER_CLOCKWISE, CLOSEST;

        public static Direction stringToDirection(String string) {
            switch (string.toUpperCase()) {
                case "CW":
                case "CLOCKWISE":
                    return CLOCKWISE;
                case "CCW":
                case "COUNTERCLOCKWISE":
                    return COUNTER_CLOCKWISE;
                default:
                    return CLOSEST;
            }
        }
    }

    protected Robot robot;
    protected PIDController pidController;
    protected Angle turnAngle;
    protected Angle currentAngle;
    protected double cumulativeDegrees;
    protected double powerFactor;
    protected double basePower;
    protected Angle errorRange;

    public Turn(Command command) {
        this.robot = Robot.getInstance();
        turnAngle = command.getAngle("angle", 0);
        powerFactor = command.getDouble("power", 1.0);
        errorRange = command.getAngle("error", 10, AngleUnit.DEGREES);
        basePower = command.getDouble("base power", 0.1);
        pidController = new PIDController(command, turnAngle.getDegrees());
    }

    @Override
    protected void onRun() {
        cumulativeDegrees = 0;
        currentAngle = robot.imu.getHeading();
        AutoRunner.log("AngleToTurn", turnAngle.getDegrees());
        AutoRunner.log("CurrentAngle", currentAngle.getDegrees());
    }

    @Override
    protected boolean runIsComplete() {
        double error = Math.abs(cumulativeDegrees - turnAngle.getDegrees());
        return error < errorRange.getDegrees();
    }

    @Override
    protected void insideRun() {
        double power = getCorrectedPower() * powerFactor;
        robot.driveTrain.drive(new Pose(0, 0, power));
        AutoRunner.log("TurnPower", power);
        AutoRunner.log("Angle", currentAngle.getDegrees());
    }

    private double getCorrectedPower() {
        Angle previousAngle = currentAngle;
        currentAngle = robot.imu.getHeading();
        double deltaDegrees = currentAngle.getDegrees() - previousAngle.getDegrees();
        if (deltaDegrees > 180) {
            deltaDegrees -= 360;
        } else if (deltaDegrees < -180) {
            deltaDegrees += 360;
        }
        cumulativeDegrees += deltaDegrees;
        double pidCorrectedPower = pidController.getCorrectedOutput(cumulativeDegrees);
        AutoRunner.log("CumulativeDegrees", cumulativeDegrees);
        AutoRunner.log("PIDPower", pidCorrectedPower);
        return GeneralMath.clipPower(pidCorrectedPower, basePower);
    }

    @Override
    protected void onEndRun() {
        robot.driveTrain.stop();
    }
}
