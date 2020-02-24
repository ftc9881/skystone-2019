package org.firstinspires.ftc.teamcode.auto.actions;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.SomethingBadHappened;
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
    protected double powerFactor;
    protected double basePower;
    protected Angle errorRange;
    protected double currentDegrees;

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
        currentDegrees = robot.imu.getIntegratedHeading().getDegrees();
        AutoRunner.log("AngleToTurn", turnAngle.getDegrees());
    }

    @Override
    protected boolean runIsComplete() {
        double error = Math.abs(currentDegrees - turnAngle.getDegrees());
        return error < errorRange.getDegrees();
    }

    @Override
    protected void insideRun() {
        currentDegrees = robot.imu.getIntegratedHeading().getDegrees();

        double pidCorrectedPower = pidController.getCorrectedOutput(robot.imu.getIntegratedHeading().getDegrees());
        double power = GeneralMath.clipPower(pidCorrectedPower, basePower) * powerFactor;
        robot.driveTrain.drive(new Pose(0, 0, power));
        AutoRunner.log("TurnPower", power);
        AutoRunner.log("PIDPower", pidCorrectedPower);
        AutoRunner.log("CurrentAngle", currentDegrees);
    }

    @Override
    protected void onEndRun() throws SomethingBadHappened {
        robot.driveTrain.stop();
        if (Math.abs(currentDegrees - turnAngle.getDegrees()) > 10) {
            throw new SomethingBadHappened("IMU probably died");
        }
    }
}
