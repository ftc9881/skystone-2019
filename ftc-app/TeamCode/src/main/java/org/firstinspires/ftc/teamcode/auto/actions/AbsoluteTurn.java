package org.firstinspires.ftc.teamcode.auto.actions;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;

public class AbsoluteTurn extends Turn {

    private PIDController pidController;
    private Angle currentAngle;
    private double cumulativeDegrees;
    private Direction direction;


    public AbsoluteTurn(Command command) {
        super(command);
        direction = Turn.Direction.stringToDirection(command.getString("direction", "CLOSEST"));
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
        double error = Math.abs(currentAngle.getDegrees() - turnAngle.getDegrees());
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
        } else if (deltaDegrees < 180) {
            deltaDegrees += 360;
        }
        cumulativeDegrees += deltaDegrees;
        double pidCorrectedPower = pidController.getCorrectedOutput(cumulativeDegrees);
        return Range.clip(Math.abs(pidCorrectedPower), basePower, 1.0);
    }

    @Override
    protected void onEndRun() {
        robot.driveTrain.stop();
    }
}
