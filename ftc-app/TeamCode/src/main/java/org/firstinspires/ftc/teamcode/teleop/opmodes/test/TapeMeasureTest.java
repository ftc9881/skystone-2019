package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp(group="Test")
//@Disabled
public class TapeMeasureTest extends TeleOpBase {

    private CRServo tapeServo;

    @Override
    protected void initialize() {
        robot.driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        tapeServo = hardwareMap.crservo.get("tape");
    }

    @Override
    protected void update() {
        tapeServo.setPower(gamepad1.right_stick_y);
    }

}
