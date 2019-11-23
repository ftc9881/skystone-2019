package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    public enum State {
        NOTHING(0, 0),
        OUT(0,0.2),
        OUT_AND_DOWN(0,0),
        IN(-1000, -0.2),
        UP_AND_OUT(-1000, 0.2);

        final int liftPosition;
        final double swivelPower;

        State(int liftPosition, double swivelPower) {
            this.liftPosition = liftPosition;
            this.swivelPower = swivelPower;
        }
    }


    public DcMotor liftMotor;
    public DcMotor swivelMotor;

    public Arm(HardwareMap hardwareMap) {

        liftMotor = hardwareMap.dcMotor.get("lift");
        swivelMotor = hardwareMap.dcMotor.get("swivel");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        swivelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        swivelMotor.setDirection(DcMotor.Direction.FORWARD);

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        swivelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setPower(0);
        swivelMotor.setPower(0);
    }

    public void move(State state) {
        if (state == State.NOTHING) return;

        swivelMotor.setPower(state.swivelPower);

        liftMotor.setTargetPosition(state.liftPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.7);
    }

    public boolean moveIsDone() {
        return !liftMotor.isBusy();
    }

    public void stop() {
        liftMotor.setPower(0);
        swivelMotor.setPower(0);
    }

}
