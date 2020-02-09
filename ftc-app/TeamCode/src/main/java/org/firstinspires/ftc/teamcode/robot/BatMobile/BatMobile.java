package org.firstinspires.ftc.teamcode.robot.BatMobile;

import org.firstinspires.ftc.teamcode.robot.Intake;
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
    public SideArm sideArm;
    public DifferentialElevator elevator;

    public ToggleServo backDepositServo;
    public ToggleServo frontDepositServo;
    public ToggleServo leftFoundationServo;
    public ToggleServo rightFoundationServo;

    private BatMobile(Robot robot) {
        this.opMode = robot.opMode;
        this.hardwareMap = robot.hardwareMap;
        this.imu = robot.imu;
        this.driveTrain = robot.driveTrain;

        sideArm = new SideArm(hardwareMap);
        elevator = new DifferentialElevator(hardwareMap);
        intake = new Intake(hardwareMap);

        backDepositServo = new ToggleServo(hardwareMap, "back deposit");
        frontDepositServo = new ToggleServo(hardwareMap, "front deposit");
        leftFoundationServo = new ToggleServo(hardwareMap, "left foundation");
        rightFoundationServo = new ToggleServo(hardwareMap, "right foundation");
    }

}
