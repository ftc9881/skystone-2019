package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.math.VelocityMotor;

public class Elevator {

    public VelocityMotor leftLift;
    public VelocityMotor rightLift;

    public Elevator(HardwareMap hardwareMap) {
        leftLift = new VelocityMotor(hardwareMap.dcMotor.get("left lift"));
        rightLift = new VelocityMotor(hardwareMap.dcMotor.get("right lift"));

        leftLift.motor.setDirection(DcMotor.Direction.FORWARD);
        rightLift.motor.setDirection(DcMotor.Direction.REVERSE);

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

    public void setPower(double left, double right) {
        leftLift.motor.setPower(left);
        rightLift.motor.setPower(right);
    }

}
