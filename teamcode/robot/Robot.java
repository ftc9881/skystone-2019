package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;

public class Robot {

    // Singleton pattern; constructor is private and enable only one instance at a time
    private static Robot instance;

    public static Robot newInstance(OpMode opMode) {
        instance = new Robot(opMode);
        return instance;
    }

    public static Robot getInstance() {
        return instance;
    }


    private AngleUnit angleUnit = AngleUnit.RADIANS;
    private OpMode opMode;

    public HardwareMap hardwareMap;

    public Vuforia vuforia;

    public DriveTrain driveTrain;
    public Arm arm;
    public Intake intake;
    public FoundationGrabber foundationGrabber;
    public SensorSystem sensorSystem;
    public OdometrySystem odometrySystem;

    public Pose currentPose;

    private BNO055IMU imu;

    private Robot(OpMode opMode) {
        this.opMode = opMode;
        hardwareMap = opMode.hardwareMap;
        vuforia = new Vuforia(hardwareMap.appContext);

        RobotSystemFactory robotFactory = new RobotSystemFactory(opMode.hardwareMap);
        driveTrain = robotFactory.driveTrain();
        arm = robotFactory.arm();
        intake = robotFactory.intakeSystem();
        foundationGrabber = robotFactory.foundationGrabber();
        sensorSystem = robotFactory.sensorSystem();
        odometrySystem = robotFactory.odometrySystem();
    }

    public void initializeImu(AngleUnit angleUnit) {
        this.angleUnit = angleUnit;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        BNO055IMU.AngleUnit imuAngleUnit = angleUnit == AngleUnit.DEGREES ? BNO055IMU.AngleUnit.DEGREES : BNO055IMU.AngleUnit.RADIANS;
        parameters.angleUnit = imuAngleUnit;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public Angle getImuHeading() {
        return new Angle(imu.getAngularOrientation().firstAngle, angleUnit);
    }

}
