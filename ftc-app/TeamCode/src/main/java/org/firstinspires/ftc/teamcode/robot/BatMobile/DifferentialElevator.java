package org.firstinspires.ftc.teamcode.robot.BatMobile;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.hardware.motor.CachingMotorEx;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;
import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;

public class DifferentialElevator {

    public CachingMotorEx left;
    public CachingMotorEx right;
    private Command pidConfig;
    private double minPower;

    private PIDController positionPid;

    DifferentialElevator(HardwareMap hardwareMap) {
        left = new CachingMotorEx(hardwareMap.dcMotor.get("left lift"));
        right = new CachingMotorEx(hardwareMap.dcMotor.get("right lift"));

        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pidConfig = new Configuration("TeleOp");
        minPower = pidConfig.getDouble("elevator min power", 0.25);
    }

    public void setPidConfig(Command pidConfig) {
        this.pidConfig = pidConfig;
        minPower = pidConfig.getDouble("elevator min power", 0.25);
    }

    public int getClicksDifference() {
        return right.getCurrentPosition() - left.getCurrentPosition();
    }

    public void setRunToRelativePosition(int liftClicks) {
        int target = left.getCurrentPosition() + liftClicks;
        positionPid = new PIDController(pidConfig, "elevator", target);
        AutoRunner.log("Elevator:Target", target);
    }

    public void updateRunToRelativePosition() {
        if (positionPid != null) {
            int currentPosition = left.getCurrentPosition();
//            double power = Range.clip(positionPid.getCorrectedOutput(currentPosition), minPower, 1);
            double power = GeneralMath.clipPower(-positionPid.getCorrectedOutput(currentPosition) + minPower);
            setPowerLE(power, 0);
        }
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
        setPowerLR(lift + extend, lift - extend, powerFactor);
    }

    private void checkAndSetMode(DcMotor.RunMode mode) {
        left.checkAndSetMode(mode);
        right.checkAndSetMode(mode);
    }

}
