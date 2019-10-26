package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * The Robot class has references to all the hardware devices.
 *
 * @author Trinity Chung
 * @version 0.0
 */
public class Robot {

    private LinearOpMode opMode;

    public DcMotor lf;
    public DcMotor rf;
    public DcMotor lb;
    public DcMotor rb;

    public DcMotor rightOdometry;
    public DcMotor leftOdometry;
    public DcMotor centerOdometry;

    public BNO055IMU imu;
    public MaxSonarI2CXL sonarSensor;


    // for locating files
    public Context fileContext;

    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;

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
        leftOdometry = lf;
        rightOdometry = rf;
        centerOdometry = lb;


        // Custom sonar device
        sonarSensor = opMode.hardwareMap.get(MaxSonarI2CXL.class, "sonar");
        sonarSensor.setI2cAddress(I2cAddr.create8bit(0xE0));
//        sonarSensor.startAutoPing(100);


        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        fileContext = opMode.hardwareMap.appContext;
    }

    public void drive(double x, double y, double r) {
        drive(x, y, r, 1);
    }

    public void drive(double x, double y, double r, double powerFactor) {
        // Calculate Power
        double lfp = Range.clip(x - r + y, -1.0, 1.0);
        double rfp = Range.clip(x + r - y, -1.0, 1.0);
        double lbp = Range.clip(x - r - y, -1.0, 1.0);
        double rbp = Range.clip(x + r + y, -1.0, 1.0);

        lf.setPower(lfp * powerFactor);
        rf.setPower(rfp * powerFactor);
        lb.setPower(lbp * powerFactor);
        rb.setPower(rbp * powerFactor);
    }


    public void drive(Pose pose) {
        drive(pose.x, pose.y, pose.r);
    }

    public void drive(Pose pose, double powerFactor) {
        drive(pose.x, pose.y, pose.r, powerFactor);
    }

    public void log(String message, boolean showTelemetry) {
        RobotLog.d(message);
        if (showTelemetry) {
            opMode.telemetry.addData("%%%LOG", message);
            opMode.telemetry.update();
        }
    }

    public void log(String message) {
        log(message, true);
    }
}
