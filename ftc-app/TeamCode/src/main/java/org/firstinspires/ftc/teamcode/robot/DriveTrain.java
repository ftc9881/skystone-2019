package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.hardware.motor.CachingMotor;

import java.util.ArrayList;
import java.util.List;

public class DriveTrain {

    public DcMotor lf;
    public DcMotor rf;
    public DcMotor lb;
    public DcMotor rb;

    /*
        CD: Try to have a consistent object life cycle across all Robot and "component" classes.  That is,
            you should have the constructor set up the original fields, etc.  Then have methods like:
                initialize() - do things like setMode(. . .), only called once after constructing
                start() - called right before operating
                update() - called while operating (driving, etc.)
                finish() - called at completion of operation
                terminate() - called once, as a bookend to initialize() to free up resources
            Objects that are containers of other objects (such as Robot -> DriveTrain) would propagate
                calls to each of their contained objects.
            Objects that have specialized behaviors (such as drive() here) should have a corresponding
                method in the containing object (i.e. Robot.drive()) which forms the interface which
                invokes drive() here
     */

    public DriveTrain(HardwareMap hardwareMap) {
        lf = new CachingMotor(hardwareMap, "lf");
        rf = new CachingMotor(hardwareMap, "rf");
        lb = new CachingMotor(hardwareMap, "lb");
        rb = new CachingMotor(hardwareMap, "rb");

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
