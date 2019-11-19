package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.structure.Action;

public class Arm {

    public enum State {
        OUT(7000),
        IN(0);

        private int swivelPosition;

        State(int position) {
            this.swivelPosition = position;
        }

        public int getSwivelPosition() {
            return swivelPosition;
        }
    }

    private static final double POWER = 1.0;

    public DcMotor liftMotor;
    public DcMotor swivelMotor;

    public Arm(HardwareMap hardwareMap) {

        liftMotor = hardwareMap.dcMotor.get("lift");
        swivelMotor = hardwareMap.dcMotor.get("swivel");

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        swivelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        swivelMotor.setDirection(DcMotor.Direction.FORWARD);

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        swivelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setPower(0);
        swivelMotor.setPower(0);
    }

    public void move(State state) {
        swivelMotor.setTargetPosition(state.getSwivelPosition());
        swivelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        swivelMotor.setPower(POWER);

        liftMotor.setPower(-POWER);
    }

    public boolean moveIsDone() {
        return !swivelMotor.isBusy();
    }

    public void stop() {
        liftMotor.setPower(0);
        swivelMotor.setPower(0);
    }

}
