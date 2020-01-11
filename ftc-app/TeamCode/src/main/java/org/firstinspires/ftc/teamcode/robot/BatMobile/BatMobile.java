package org.firstinspires.ftc.teamcode.robot.BatMobile;

import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.devices.ToggleServo;

public class BatMobile extends Robot {

    // Singleton pattern; constructor is private and enable only one instance at a time
    private static BatMobile instance;
    public static BatMobile getInstance() {
        Robot robot = Robot.getInstance();
        if (instance == null && robot != null) {
            instance = new BatMobile(robot);
        }
        return instance;
    }

    public Intake intake;
    public SideArm sideArm;
    public SimpleElevator elevator;
    public DifferentialElevator differentialElevator;

    public ToggleServo leftFoundationServo;
    public ToggleServo rightFoundationServo;
    public ToggleServo backDepositServo;
    public ToggleServo frontDepositServo;
    public ToggleServo capstoneServo;

    private BatMobile(Robot robot) {
        this.opMode = robot.opMode;
        this.hardwareMap = robot.hardwareMap;
        this.imu = robot.imu;
        this.driveTrain = robot.driveTrain;

        sideArm = new SideArm(hardwareMap);
        elevator = new SimpleElevator(hardwareMap);
        intake = new Intake(hardwareMap);

        leftFoundationServo = new ToggleServo(hardwareMap, "left foundation");
        rightFoundationServo = new ToggleServo(hardwareMap, "right foundation");
        backDepositServo = new ToggleServo(hardwareMap, "back deposit");
        frontDepositServo = new ToggleServo(hardwareMap, "front deposit");
        capstoneServo = new ToggleServo(hardwareMap, "capstone");
    }

}
