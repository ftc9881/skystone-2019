package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class ArmBot extends Robot {

    // Singleton pattern; constructor is private and enable only one instance at a time
    private static ArmBot instance;

    public static ArmBot newInstance(OpMode opMode) {
        instance = new ArmBot(opMode);
        return instance;
    }

    public static ArmBot getInstance() {
        return instance;
    }

    public ToggleServo stoneServo;
    public ToggleServo capstoneServo;
    public ToggleServo foundationServo;

    public Arm arm;
    public Intake intake;
    public SensorSystem sensorSystem;
    public OdometrySystem odometrySystem;

    private ArmBot(OpMode opMode) {
        super(opMode);

        RobotSystemFactory robotFactory = new RobotSystemFactory(opMode.hardwareMap);
        visionSystem = robotFactory.visionSystem();
        driveTrain = robotFactory.driveTrain();
        arm = robotFactory.arm();
        intake = robotFactory.intakeSystem();
        sensorSystem = robotFactory.sensorSystem();
        odometrySystem = robotFactory.odometrySystem();

        stoneServo = new ToggleServo(hardwareMap, "stone");
        capstoneServo = new ToggleServo(hardwareMap, "capstone");
        foundationServo = new ToggleServo(hardwareMap, "foundation");
    }

}
