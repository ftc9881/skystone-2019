package org.firstinspires.ftc.teamcode;

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
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
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
