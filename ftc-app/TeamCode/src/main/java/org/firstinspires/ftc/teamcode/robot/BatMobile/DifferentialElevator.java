package org.firstinspires.ftc.teamcode.robot.BatMobile;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.motor.CachingMotorEx;

public class DifferentialElevator {

    public CachingMotorEx left;
    public CachingMotorEx right;

    public DifferentialElevator(HardwareMap hardwareMap) {
        left = new CachingMotorEx(hardwareMap.dcMotor.get("left lift"));
        right = new CachingMotorEx(hardwareMap.dcMotor.get("right lift"));

        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.FORWARD);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runToRelativePosition(int liftClicks, int extendClicks) {
        int leftClicks = left.getCurrentPosition() + extendClicks + liftClicks;
        int rightClicks = right.getCurrentPosition() + extendClicks - liftClicks;
        left.setTargetPosition(leftClicks);
        right.setTargetPosition(rightClicks);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPowerLR(1, 1);
    }

    public void setPowerLR(double left, double right, double powerFactor) {
        checkAndSetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.left.setPower(left * powerFactor);
        this.right.setPower(right * powerFactor);
    }

    public void setPowerLR(double left, double right) {
        setPowerLR(left, right, 1.0);
    }

    public void setPowerLE(double lift, double extend) {
        setPowerLE(lift, extend, 1.0);
    }

    public void setPowerLE(double lift, double extend, double powerFactor) {
        setPowerLR(extend + lift, extend - lift, powerFactor);
    }

    private void checkAndSetMode(DcMotor.RunMode mode) {
        if (left.getMode() != mode) {
            left.setMode(mode);
        }
        if (right.getMode() != mode) {
            right.setMode(mode);
        }
    }

}
