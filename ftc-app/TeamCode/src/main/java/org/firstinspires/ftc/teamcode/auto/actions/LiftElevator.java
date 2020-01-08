package org.firstinspires.ftc.teamcode.auto.actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.AutoOpConfiguration;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.robot.Elevator;
import org.firstinspires.ftc.teamcode.robot.Robot;


public class LiftElevator extends Action {

    Robot robot;
    private PIDController lPID;
    private PIDController rPID;
    private double lTargetClicks;
    private double rTargetClicks;
    private double lClicks;
    private double rClicks;
    private double powerFactor;
    private int startingClicks;
    private int errorRange;
    private Elevator elevator;

    public LiftElevator(int clicks, double powerFactor) {
        this.robot = Robot.getInstance();
        this.powerFactor = powerFactor;
        elevator = robot.elevator;
        this.startingClicks = elevator.leftLift.motor.getCurrentPosition();
        this.lTargetClicks = elevator.leftLift.motor.getCurrentPosition() + clicks;
        this.rTargetClicks = elevator.rightLift.motor.getCurrentPosition() + clicks;
        AutoOpConfiguration config = AutoOpConfiguration.getInstance();
        errorRange = config.properties.getInt("elevator clicks error", 30);
    }

    @Override
    protected void onRun() {
        elevator.leftLift.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.rightLift.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lClicks = elevator.leftLift.motor.getCurrentPosition();
        rClicks = elevator.rightLift.motor.getCurrentPosition();
        AutoRunner.log("Left  Clicks", lClicks);
        AutoRunner.log("Right Clicks", rClicks);

    }

    @Override
    protected boolean runIsComplete() {
        double errorClicks = Math.abs(elevator.leftLift.motor.getCurrentPosition() - lTargetClicks);
        return errorClicks < errorRange;
    }

    @Override
    protected void insideRun() {
        elevator.rightLift.motor.setPower(powerFactor);
        elevator.leftLift.motor.setPower(powerFactor);
        AutoRunner.log("=============");
        AutoRunner.log("Left  Clicks", lClicks);
        AutoRunner.log("Right Clicks", rClicks);
    }

    @Override
    protected void onEndRun() {
        robot.driveTrain.stop();
    }
}
