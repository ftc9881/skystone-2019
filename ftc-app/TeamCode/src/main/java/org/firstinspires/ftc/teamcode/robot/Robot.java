package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.vision.OpenCV;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;

public class Robot {

    // Singleton pattern; constructor is private and enable only one instance at a time
    private static Robot instance;

    public static Robot newInstance(LinearOpMode opMode) {
        instance = new Robot(opMode);
        return instance;
    }

    public static Robot getInstance() {
        return instance;
    }


    private AngleUnit angleUnit = AngleUnit.RADIANS;
    public LinearOpMode opMode;

    public HardwareMap hardwareMap;

    public DriveTrain driveTrain;

    public VisionSystem visionSystem;
    public SensorSystem sensorSystem;
    public OdometrySystem odometrySystem;

    public Pose currentPose;

    public BNO055IMU imu;

    protected Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        opMode.msStuckDetectInit = 3000;
        opMode.msStuckDetectInitLoop = 3000;

        hardwareMap = opMode.hardwareMap;

        RobotSystemFactory robotFactory = new RobotSystemFactory(opMode.hardwareMap);
        visionSystem = robotFactory.visionSystem();
        sensorSystem = robotFactory.sensorSystem();
        odometrySystem = robotFactory.odometrySystem();
        driveTrain = robotFactory.driveTrain();
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
        if (imu == null) {
            initializeImu(angleUnit);
        }
        return new Angle(imu.getAngularOrientation().firstAngle, angleUnit);
    }

}
