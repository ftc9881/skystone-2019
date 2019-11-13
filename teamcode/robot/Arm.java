package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm implements IRobotSystem {

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

}
