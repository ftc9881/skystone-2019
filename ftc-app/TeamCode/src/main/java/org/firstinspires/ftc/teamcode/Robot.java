package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

    public BNO055IMU imu;

//    Servo someServo;

    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;

        lf = opMode.hardwareMap.dcMotor.get("front_left");
        rf = opMode.hardwareMap.dcMotor.get("front_right");
        lb = opMode.hardwareMap.dcMotor.get("back_left");
        rb = opMode.hardwareMap.dcMotor.get("back_right");

//        someServo = opMode.hardwareMap.servo.get("servo");

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
    }


    public void log(String tag, String message, boolean showTelemetry) {
        RobotLog.ee(tag, message);
        if (showTelemetry) {
            opMode.telemetry.addData(tag, message);
            opMode.telemetry.update();
        }
    }

    public void log(String tag, String message) {
        log(tag, message, true);
    }
}
