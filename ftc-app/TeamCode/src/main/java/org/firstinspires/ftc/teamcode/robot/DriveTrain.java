package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.hardware.motor.CachingMotor;

public class DriveTrain {

    public DcMotor lf;
    public DcMotor rf;
    public DcMotor lb;
    public DcMotor rb;

    public DriveTrain(HardwareMap hardwareMap) {
        lf = new CachingMotor(hardwareMap, "lf");
        rf = new CachingMotor(hardwareMap, "rf");
        lb = new CachingMotor(hardwareMap, "lb");
        rb = new CachingMotor(hardwareMap, "rb");
//        lf = hardwareMap.dcMotor.get("lf");
//        rf = hardwareMap.dcMotor.get("rf");
//        lb = hardwareMap.dcMotor.get("lb");
//        rb = hardwareMap.dcMotor.get("rb");


        lf.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.REVERSE);

        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMode(DcMotor.RunMode mode) {
        lf.setMode(mode);
        lb.setMode(mode);
        rf.setMode(mode);
        rb.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        lf.setZeroPowerBehavior(behavior);
        rf.setZeroPowerBehavior(behavior);
        lb.setZeroPowerBehavior(behavior);
        rb.setZeroPowerBehavior(behavior);
    }

    public void drive(Pose pose) {
        drive(pose, 1.0);
    }
    public void drive(Pose pose, double powerFactor) {
        // Calculate power for mecanum drive
        double lfp = Range.clip(pose.y + pose.r + pose.x, -1.0, 1.0);
        double rfp = Range.clip(pose.y - pose.r - pose.x, -1.0, 1.0);
        double lbp = Range.clip(pose.y + pose.r - pose.x, -1.0, 1.0);
        double rbp = Range.clip(pose.y - pose.r + pose.x, -1.0, 1.0);

        lf.setPower(lfp * powerFactor);
        rf.setPower(rfp * powerFactor);
        lb.setPower(lbp * powerFactor);
        rb.setPower(rbp * powerFactor);
    }

    public void stop() {
        drive(new Pose(0, 0, 0));
    }

}
