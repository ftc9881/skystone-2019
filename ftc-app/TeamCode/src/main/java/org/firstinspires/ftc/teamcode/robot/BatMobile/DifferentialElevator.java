package org.firstinspires.ftc.teamcode.robot.BatMobile;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.devices.VelocityMotor;
import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;

public class DifferentialElevator {

    private int clicksPerLiftLevel;
    private int clicksPerExtendLevel;

    public VelocityMotor leftLift;
    public VelocityMotor rightLift;

    public DifferentialElevator(HardwareMap hardwareMap) {
        Configuration config = new Configuration("HardwareConstants");
        clicksPerLiftLevel = config.getInt("clicks per lift level", 0);
        clicksPerExtendLevel = config.getInt("clicks per extend level", 0);

        leftLift = new VelocityMotor(hardwareMap.dcMotor.get("left lift"));
        rightLift = new VelocityMotor(hardwareMap.dcMotor.get("right lift"));

        leftLift.motor.setDirection(DcMotor.Direction.FORWARD);
        rightLift.motor.setDirection(DcMotor.Direction.FORWARD);

        leftLift.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftLift.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void updateVelocity() {
        leftLift.update();
        rightLift.update();
    }

    public void setPowerLR(double left, double right) {
        leftLift.motor.setPower(left);
        rightLift.motor.setPower(right);
    }

    public void setPowerLE(double lift, double extend) {
        setPowerLE(lift, extend, 1.0);
    }

    public void setPowerLE(double lift, double extend, double powerFactor) {
        double leftPower = lift + extend;
        double rightPower = extend - lift;
        leftLift.motor.setPower(leftPower * powerFactor);
        rightLift.motor.setPower(rightPower * powerFactor);
    }

    public void relativeLiftToLevel(int skystoneLevel) {
        int leftTargetClicks = leftLift.motor.getCurrentPosition() + skystoneLevel * clicksPerLiftLevel;
        int rightTargetClicks = rightLift.motor.getCurrentPosition() + skystoneLevel * clicksPerLiftLevel;
        leftLift.motor.setTargetPosition(leftTargetClicks);
        rightLift.motor.setTargetPosition(rightTargetClicks);
        leftLift.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPowerLR(1.0, 1.0);
    }

    public void relativeExtendToLevel(int skystoneLevel) {
        int leftTargetClicks = leftLift.motor.getCurrentPosition() + skystoneLevel * clicksPerExtendLevel;
        int rightTargetClicks = rightLift.motor.getCurrentPosition() + skystoneLevel * clicksPerExtendLevel;
        leftLift.motor.setTargetPosition(leftTargetClicks);
        rightLift.motor.setTargetPosition(rightTargetClicks);
        leftLift.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPowerLR(1.0, 1.0);
    }

}
