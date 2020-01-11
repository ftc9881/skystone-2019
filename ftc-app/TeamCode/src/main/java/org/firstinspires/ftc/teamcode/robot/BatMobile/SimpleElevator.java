package org.firstinspires.ftc.teamcode.robot.BatMobile;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;

public class SimpleElevator {

    public DcMotor lift;
    public DcMotor extend;
    private int clicksPerLiftLevel;
    private int clicksPerExtendLevel;

    public SimpleElevator(HardwareMap hardwareMap) {
        Configuration config = new Configuration("HardwareConstants");
        clicksPerLiftLevel = config.getInt("clicks per lift level", 0);
        clicksPerExtendLevel = config.getInt("clicks per extend level", 0);

        lift = hardwareMap.dcMotor.get("left lift");
        extend = hardwareMap.dcMotor.get("right lift");

        lift.setDirection(DcMotor.Direction.FORWARD);
        extend.setDirection(DcMotor.Direction.REVERSE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void relativeLiftToLevel(int skystoneLevel) {
        int targetClicks = lift.getCurrentPosition() + skystoneLevel * clicksPerLiftLevel;
        lift.setTargetPosition(targetClicks);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1.0);
    }

    public void relativeExtendToLevel(int skystoneLevel) {
        int targetClicks = extend.getCurrentPosition() + skystoneLevel * clicksPerExtendLevel;
        extend.setTargetPosition(targetClicks);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extend.setPower(1.0);
    }
}
