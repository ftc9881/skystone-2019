package org.firstinspires.ftc.teamcode.robot.ArmBot;

import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.hardware.servo.ToggleServo;

public class ArmBot extends Robot {

    // Singleton pattern; constructor is private and enable only one instance at a time
    private static ArmBot instance;

    public static ArmBot getInstance() {
        Robot robot = Robot.getInstance();
        if (instance == null && robot != null) {
            instance = new ArmBot(robot);
        }
        return instance;
    }

    public ToggleServo stoneServo;
    public ToggleServo capstoneServo;
    public ToggleServo foundationServo;

    public Arm arm;
    public Intake intake;

    private ArmBot(Robot robot) {
        this.opMode = robot.opMode;
        this.hardwareMap = robot.hardwareMap;
        this.imu = robot.imu;
        this.driveTrain = robot.driveTrain;

        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);

        stoneServo = new ToggleServo(hardwareMap, "stone");
        capstoneServo = new ToggleServo(hardwareMap, "capstone");
        foundationServo = new ToggleServo(hardwareMap, "foundation");
    }

}
