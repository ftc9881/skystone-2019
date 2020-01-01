package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TestElevatorDrive extends BaseDrive {

    DcMotor leftLift;
    DcMotor rightLift;

    @Override
    protected void initialize() {
        super.initialize();

        leftLift = hardwareMap.dcMotor.get("left lift");
        rightLift = hardwareMap.dcMotor.get("right lift");
    }

    @Override
    protected void update() {
        super.update();

        leftLift.setPower(-gamepad2.left_stick_y);
        rightLift.setPower(-gamepad2.right_stick_y);

        telemetry.addData("Player 1", "Drive Control");
        telemetry.addData("Player 2", "Lift Control (left/right stick y)");
        telemetry.addData("left lift encoder", leftLift.getCurrentPosition());
        telemetry.addData("right lift encoder", rightLift.getCurrentPosition());
        telemetry.update();
    }

}
