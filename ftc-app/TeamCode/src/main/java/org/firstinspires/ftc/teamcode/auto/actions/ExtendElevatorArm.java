package org.firstinspires.ftc.teamcode.auto.actions;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto.structure.AutoOpConfiguration;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.robot.BatMobile.Elevator;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;

public class ExtendElevatorArm extends Action {

    Robot robot;
    private PIDController lPID;
    private PIDController rPID;
    private double targetClicks;
    private double lClicks;
    private double rClicks;
    private double powerFactor;
    private double basePower;
    private int errorRange;
    private Elevator elevator;

    public ExtendElevatorArm(int clicks, double powerFactor) {
        this.robot = Robot.getInstance();
        this.powerFactor = powerFactor;
        elevator = robot.elevator;

        AutoOpConfiguration config = AutoOpConfiguration.getInstance();
        errorRange = config.properties.getInt("elevator clicks error", 20);
        double kP = config.properties.getDouble("lift kp", 0);
        double kI = config.properties.getDouble("lift ki", 0);
        double kD = config.properties.getDouble("lift kd", 0);
        lPID = new PIDController(kP ,kI ,kD, elevator.leftLift.motor.getTargetPosition()+clicks);
        rPID = new PIDController(kP ,kI ,kD, elevator.rightLift.motor.getTargetPosition()-clicks);
    }

    @Override
    protected void onRun() {
        lClicks = elevator.leftLift.motor.getCurrentPosition();
        rClicks = elevator.rightLift.motor.getCurrentPosition();
        AutoRunner.log("Left  Clicks", lClicks);
        AutoRunner.log("Right Clicks", rClicks);

    }

    @Override
    protected boolean runIsComplete() {
        double errorClicks = Math.abs(Math.abs(lClicks-rClicks)/2 - targetClicks);
        return errorClicks < errorRange;
    }

    @Override
    protected void insideRun() {
        lClicks = elevator.leftLift.motor.getCurrentPosition();
        rClicks = elevator.rightLift.motor.getCurrentPosition();
        double lPower = Range.clip(lPID.getCorrectedOutput(lClicks),-1, 1)*powerFactor;
        double rPower = Range.clip(rPID.getCorrectedOutput(rClicks),-1, 1)*powerFactor;
        elevator.rightLift.motor.setPower(rPower);
        elevator.leftLift.motor.setPower(lPower);
        AutoRunner.log("=============");
        AutoRunner.log("Left  Clicks", lClicks);
        AutoRunner.log("Right Clicks", rClicks);
        AutoRunner.log("Left Power", lPower);
        AutoRunner.log("Right Power", rPower);
    }

    @Override
    protected void onEndRun() {
        robot.driveTrain.stop();
    }
}
