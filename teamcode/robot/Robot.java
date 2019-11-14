package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.math.Pose;

/**
 * The Robot class has references to all the hardware devices.
 *
 * @author Trinity Chung
 * @version 0.0
 */
public class Robot {

    // TODO: Singleton pattern
//    private static Robot instance;
//
//    public static Robot newInstance(OpMode opMode) {
//        instance = new Robot(opMode);
//        return instance;
//    }
//
//    public static Robot getInstance() {
//        return instance;
//    }

    private OpMode opMode;
    private RobotSystemFactory robotFactory;

    public Vuforia vuforia;

    public DriveTrain driveTrain;
    public Arm arm;
    public Intake intake;
    public FoundationGrabber foundationGrabber;
    public SensorSystem sensorSystem;
    public OdometrySystem odometrySystem;

    public Pose currentPose;

    private BNO055IMU imu;

    public Robot(OpMode opMode) {
        this.opMode = opMode;
        vuforia = new Vuforia(opMode.hardwareMap.appContext);
        robotFactory = new RobotSystemFactory(opMode.hardwareMap);
        driveTrain = robotFactory.driveTrain();
        arm = robotFactory.arm();
        intake = robotFactory.intakeSystem();
        foundationGrabber = robotFactory.foundationGrabber();
        sensorSystem = robotFactory.sensorSystem();
        odometrySystem = robotFactory.odometrySystem();

        initializeImu();
    }

    private void initializeImu() {
        // IMU Parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void drive(Pose pose) {
        drive(pose.x, pose.y, pose.r);
    }
    public void drive(Pose pose, double powerFactor) {
        drive(pose.x, pose.y, pose.r, powerFactor);
    }
    public void drive(double x, double y, double r) {
        drive(x, y, r, 1);
    }
    public void drive(double x, double y, double r, double powerFactor) {
        // Calculate power for mecanum drive
        double lfp = Range.clip(x - r + y, -1.0, 1.0);
        double rfp = Range.clip(x + r - y, -1.0, 1.0);
        double lbp = Range.clip(x - r - y, -1.0, 1.0);
        double rbp = Range.clip(x + r + y, -1.0, 1.0);

        driveTrain.lf.setPower(lfp * powerFactor);
        driveTrain.rf.setPower(rfp * powerFactor);
        driveTrain.lb.setPower(lbp * powerFactor);
        driveTrain.rb.setPower(rbp * powerFactor);
    }

    public void stop() {
        drive(0, 0, 0);
    }

    public double getImuHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

}
