package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.sensors.MaxSonarI2CXL;
import org.firstinspires.ftc.teamcode.utility.Pose;

/**
 * The Robot class has references to all the hardware devices.
 *
 * @author Trinity Chung
 * @version 0.0
 */
public class Robot {

    // TODO: Servo positions
    private final double GRABBER_LEFT_GRAB_POSITION = 0;
    private final double GRABBER_LEFT_RELEASE_POSITION = 0;
    private final double GRABBER_RIGHT_GRAB_POSITION = 0;
    private final double GRABBER_RIGHT_RELEASE_POSITION = 0;
    private final double ARM_GRAB_POSITION = 0;
    private final double ARM_RELEASE_POSITION = 0;

    private LinearOpMode opMode;

    public DcMotor lf;
    public DcMotor rf;
    public DcMotor lb;
    public DcMotor rb;

    private DcMotor odometryRight;
    private DcMotor odometryLeft;
    private DcMotor odometryCenter;

    private Servo foundationGrabberLeft;
    private Servo foundationGrabberRight;

    private Servo armServo;

    private DcMotor elevatorLeft;
    private DcMotor elevatorRight;

    public BNO055IMU imu;
    public MaxSonarI2CXL sonarSensor;

    // for locating files (needed for Vuforia)
    public Context fileContext;


    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        this.fileContext = opMode.hardwareMap.appContext;

        lf = opMode.hardwareMap.dcMotor.get("front_left");
        rf = opMode.hardwareMap.dcMotor.get("front_right");
        lb = opMode.hardwareMap.dcMotor.get("back_left");
        rb = opMode.hardwareMap.dcMotor.get("back_right");

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lf.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);

        // The lf/rf/lb/rb wheels do not have encoders plugged in
        // Instead, the encoder is connected to the tracking wheels.
        odometryLeft = lf;
        odometryRight = rf;
        odometryCenter = lb;


        // TODO: put in robot controller configuration
//        foundationGrabberLeft = opMode.hardwareMap.servo.get("left_grabber");
//        foundationGrabberRight = opMode.hardwareMap.servo.get("right_grabber");
//
//        armServo = opMode.hardwareMap.servo.get("arm");
//
//        elevatorLeft = opMode.hardwareMap.dcMotor.get("left_elevator");
//        elevatorRight = opMode.hardwareMap.dcMotor.get("right_elevator");


        // Custom sonar device
        sonarSensor = opMode.hardwareMap.get(MaxSonarI2CXL.class, "sonar");
        sonarSensor.setI2cAddress(I2cAddr.create8bit(0xE0));
//        sonarSensor.startAutoPing(100);


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


    public double getOdometryCenterPosition() {
        return odometryCenter.getCurrentPosition();
    }
    public double getOdometryLeftPosition() {
        return odometryLeft.getCurrentPosition();
    }
    public double getOdometryRightPosition() {
        return odometryRight.getCurrentPosition();
    }


    public void grabFoundation() {
        foundationGrabberLeft.setPosition(GRABBER_LEFT_GRAB_POSITION);
        foundationGrabberRight.setPosition(GRABBER_RIGHT_GRAB_POSITION);
    }
    public void releaseFoundation() {
        foundationGrabberLeft.setPosition(GRABBER_LEFT_RELEASE_POSITION);
        foundationGrabberRight.setPosition(GRABBER_RIGHT_RELEASE_POSITION);
    }

    public void grabStone() {
        armServo.setPosition(ARM_GRAB_POSITION);
    }

    public void placeStone() {
        armServo.setPosition(ARM_RELEASE_POSITION);
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

        lf.setPower(lfp * powerFactor);
        rf.setPower(rfp * powerFactor);
        lb.setPower(lbp * powerFactor);
        rb.setPower(rbp * powerFactor);
    }

    public void stop() {
        drive(0, 0, 0);
    }

    public void logTelemetry(String message) {
        RobotLog.d(message);
        opMode.telemetry.addData("%%%LOG", message);
        opMode.telemetry.update();
    }
}
