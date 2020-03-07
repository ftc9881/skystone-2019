package org.firstinspires.ftc.teamcode.robot.BatMobile;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.hardware.motor.OdometryWheel;
import org.firstinspires.ftc.teamcode.hardware.sensor.IDistanceSensor;
import org.firstinspires.ftc.teamcode.hardware.sensor.MaxSonarAnalogSensor;
import org.firstinspires.ftc.teamcode.hardware.sensor.RevDistanceSensor;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.hardware.servo.ToggleServo;

public class BatMobile extends Robot {

    private static BatMobile instance;

    public static BatMobile getInstance() {
        Robot robot = Robot.getInstance();
        if (instance == null && robot != null) {
            instance = new BatMobile(robot);
        }
        return instance;
    }

    public static BatMobile createInstance() {
        Robot robot = Robot.getInstance();
        instance = new BatMobile(robot);
        return instance;
    }

    public Intake intake;
    public SideArm redSideArm;
    public SideArm blueSideArm;
    public DifferentialElevator elevator;

    public ToggleServo frontDepositServo;
    public ToggleServo backDepositServo;
    public ToggleServo leftFoundationServo;
    public ToggleServo rightFoundationServo;

    public OdometryWheel odometryY;
    public RevDistanceSensor leftSensor;
    public RevDistanceSensor rightSensor;

    private BatMobile(Robot robot) {
        this.opMode = robot.opMode;
        this.imu = robot.imu;
        this.driveTrain = robot.driveTrain;

        HardwareMap hardwareMap = robot.getHardwareMap();

        redSideArm = new SideArm(AutoRunner.Side.RED, hardwareMap);
        blueSideArm = new SideArm(AutoRunner.Side.BLUE, hardwareMap);
        elevator = new DifferentialElevator(hardwareMap);
        intake = new Intake(hardwareMap);

        frontDepositServo = new ToggleServo(hardwareMap, "front deposit");
        backDepositServo = new ToggleServo(hardwareMap, "back deposit");

        leftFoundationServo = new ToggleServo(hardwareMap, "left foundation");
        rightFoundationServo = new ToggleServo(hardwareMap, "right foundation");

        odometryY = new OdometryWheel(intake.right);

        leftSensor = new RevDistanceSensor(hardwareMap, "left side sensor");
        rightSensor = new RevDistanceSensor(hardwareMap, "right side sensor");
    }

    public void setToUseEncoder() {
        driveTrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        driveTrain.rb.setP;
    }

    public SideArm getSideArm() {
        AutoRunner.Side side = AutoRunner.getSide();
        return side == AutoRunner.Side.RED ? redSideArm : blueSideArm;
    }
    public SideArm getOtherSideArm() {
        AutoRunner.Side side = AutoRunner.getSide();
        return side == AutoRunner.Side.BLUE ? redSideArm : blueSideArm;
    }

    public IDistanceSensor getSideSensor() {
        return AutoRunner.getSide() == AutoRunner.Side.RED ? leftSensor : rightSensor;
    }

    public IDistanceSensor getOtherSideSensor() {
        return AutoRunner.getSide() == AutoRunner.Side.RED ? rightSensor : leftSensor;
    }

    public boolean sensorDead() {
        return leftSensor.isDead() || rightSensor.isDead();
    }

}
