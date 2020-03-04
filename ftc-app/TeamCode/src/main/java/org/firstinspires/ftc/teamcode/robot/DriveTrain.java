package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.stat.descriptive.moment.GeometricMean;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.hardware.motor.CachingMotorEx;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.hardware.motor.CachingMotor;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;
import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;

import java.util.ArrayList;
import java.util.List;

public class DriveTrain {

    public CachingMotorEx lf;
    public CachingMotorEx rf;
    public CachingMotorEx lb;
    public CachingMotorEx rb;

    private DcMotor.RunMode mode;
    private double targetVelocity = 2900;

    DriveTrain(HardwareMap hardwareMap) {
        lf = new CachingMotorEx(hardwareMap, "lf");
        rf = new CachingMotorEx(hardwareMap, "rf");
        lb = new CachingMotorEx(hardwareMap, "lb");
        rb = new CachingMotorEx(hardwareMap, "rb");

        lf.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.REVERSE);

        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void setMode(DcMotor.RunMode mode) {
        this.mode = mode;
        lf.setMode(mode);
        lb.setMode(mode);
        rf.setMode(mode);
        rb.setMode(mode);
    }

    public double getAverageClicks() {
        List<Number> clicks = new ArrayList<>();
        clicks.add(lf.getCurrentPosition());
        clicks.add(rf.getCurrentPosition());
        clicks.add(lb.getCurrentPosition());
        clicks.add(rb.getCurrentPosition());
        return GeneralMath.mean(clicks);
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
        double lfp = GeneralMath.clipPower(pose.y + pose.r + pose.x);
        double rfp = GeneralMath.clipPower(pose.y - pose.r - pose.x);
        double lbp = GeneralMath.clipPower(pose.y + pose.r - pose.x);
        double rbp = GeneralMath.clipPower(pose.y - pose.r + pose.x);

        if (mode == DcMotor.RunMode.RUN_USING_ENCODER) {
            lf.setVelocity(lfp * targetVelocity * powerFactor);
            rf.setVelocity(rfp * targetVelocity * powerFactor);
            lb.setVelocity(lbp * targetVelocity * powerFactor);
            rb.setVelocity(rbp * targetVelocity * powerFactor);
        } else {
            lf.setPower(lfp * powerFactor);
            rf.setPower(rfp * powerFactor);
            lb.setPower(lbp * powerFactor);
            rb.setPower(rbp * powerFactor);
        }
    }

    public void setVelocityPIDF() {
        Configuration config = new Configuration("HardwareConstants");
        setVelocityPIDF(config);
    }


    public void setVelocityPIDF(Command config) {
        targetVelocity = config.getDouble("dt velocity", targetVelocity);
        lf.setVelocityPIDFCoefficients(config, "dt");
        rf.setVelocityPIDFCoefficients(config, "dt");
        lb.setVelocityPIDFCoefficients(config, "dt");
        rb.setVelocityPIDFCoefficients(config, "dt");
    }

    public double getMaxTargetVelocity() {
        return targetVelocity;
    }

    public DcMotor.RunMode getMode() {
        return mode;
    }

    public void stop() {
        drive(new Pose(0, 0, 0));
    }

}
