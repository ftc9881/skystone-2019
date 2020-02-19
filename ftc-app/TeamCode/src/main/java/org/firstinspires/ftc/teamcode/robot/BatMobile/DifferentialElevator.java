package org.firstinspires.ftc.teamcode.robot.BatMobile;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.SomethingBadHappened;
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
    private RunToPosition leftRunToPositionAction;
    private RunToPosition rightRunToPositionAction;

    public DifferentialElevator(HardwareMap hardwareMap) {
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

        pidConfig = new Configuration("HardwareConstants");
        minPower = pidConfig.getDouble("elevator min power", 0.25);
//        AutoRunner.log("elevatorDefaultVelocityPidf", left.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
//
//        left.setVelocityPIDFCoefficients(pidConfig, "elevator");
//        right.setVelocityPIDFCoefficients(pidConfig, "elevator");
//
//        AutoRunner.log("elevatorOurVelocityPidf", left.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));

    }

    public void setPidConfig(Command pidConfig) {
        this.pidConfig = pidConfig;
        minPower = pidConfig.getDouble("elevator min power", 0.25);
    }

    public int getClicksDifference() {
        return right.getCurrentPosition() - left.getCurrentPosition();
    }

    public void setRunToRelativePosition(int liftClicks, int extendClicks) {
        int leftClicks = left.getCurrentPosition() + extendClicks + liftClicks;
        int rightClicks = right.getCurrentPosition() + extendClicks - liftClicks;
        AutoRunner.log("LeftTargetPosition", leftClicks);
        AutoRunner.log("RightTargetPosition", rightClicks);
        leftRunToPositionAction = new RunToPosition(left, leftClicks);
        rightRunToPositionAction = new RunToPosition(right, rightClicks);
    }

    public void startRunToRelativePosition() {
        if (leftRunToPositionAction != null) {
            leftRunToPositionAction.start();
        }
        if (rightRunToPositionAction!= null) {
            rightRunToPositionAction.start();
        }
    }

    public void stopRunToRelativePosition() {
        if (leftRunToPositionAction != null) {
            leftRunToPositionAction.stop();
        }
        if (rightRunToPositionAction!= null) {
            rightRunToPositionAction.stop();
        }
    }

    public void updateRunToRelativePosition() {
        if (leftRunToPositionAction != null) {
            leftRunToPositionAction.insideRun();
        }
        if (rightRunToPositionAction!= null) {
            rightRunToPositionAction.insideRun();
        }
    }

    @Deprecated
    public void runToRelativePositionTraditional(int liftClicks, int extendClicks) {
        cancelRunToPositionActions();
        int leftClicks = left.getCurrentPosition() + extendClicks + liftClicks;
        int rightClicks = right.getCurrentPosition() + extendClicks - liftClicks;
        AutoRunner.log("LeftTargetPosition", leftClicks);
        AutoRunner.log("RightTargetPosition", rightClicks);
        left.setTargetPosition(leftClicks);
        right.setTargetPosition(rightClicks);
        checkAndSetMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setVelocity(100);
        right.setVelocity(100);
    }

    public void setVelocity(double leftVelocity, double rightVelocity) {
        cancelRunToPositionActions();
        checkAndSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setVelocity(leftVelocity);
        right.setVelocity(rightVelocity);
    }

    public void setPowerLR(double left, double right, double powerFactor) {
        cancelRunToPositionActions();
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

    private void cancelRunToPositionActions() {
        leftRunToPositionAction = null;
        rightRunToPositionAction = null;
    }

    public class RunToPosition extends Action {
        private PIDController pid;
        private DcMotor motor;

        RunToPosition(CachingMotorEx motor, int target) {
            this.motor = motor;
            motor.checkAndSetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pid = new PIDController(pidConfig, "elevator", target);
        }

        @Override
        protected void onRun() {
        }

        @Override
        protected void insideRun() {
            int currentPosition = motor.getCurrentPosition();
            double correctedPower = Range.clip(pid.getCorrectedOutput(currentPosition), minPower, 1);
            motor.setPower(correctedPower);
        }

        @Override
        protected boolean runIsComplete() {
            return false;
        }

        @Override
        protected void onEndRun() {
//            motor.setPower(0);
        }
    }

}
