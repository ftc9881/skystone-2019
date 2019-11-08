package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Meet 0")
public class Meet0Drive extends SimpleDrive {

    @Override
    protected void initialize() {
        super.initialize();
    }

    @Override
    protected void update() {
        super.update();

        robot.arm.swivelMotor.setPower(gamepad1.left_bumper ? 0.5 : 0);
        robot.arm.swivelMotor.setPower(gamepad1.right_bumper ? -0.5 : 0);

        robot.arm.liftMotor.setPower(gamepad1.right_trigger);
        robot.arm.liftMotor.setPower(-gamepad1.left_trigger);

        robot.intake.left.setPower(gamepad1.a ? 1.0 : 0);
        robot.intake.right.setPower(gamepad1.a ? 1.0 : 0);

        robot.intake.left.setPower(gamepad1.b ? -1.0 : 0);
        robot.intake.right.setPower(gamepad1.b ? -1.0 : 0);

        telemetry.addData("Swivel", robot.arm.swivelMotor.getCurrentPosition());
        telemetry.addData("Lift", robot.arm.liftMotor.getCurrentPosition());
        telemetry.update();
    }

    private void moveArmOut() {

    }

    private void moveArmIn() {

    }

}
