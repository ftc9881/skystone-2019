package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;

@TeleOp(name = "Debug Vuforia Drive", group = "Debug")
public class DebugVuforiaDrive extends SimpleDrive {

    @Override
    protected void initialize() {
        super.initialize();

        robot.vuforia.initialize();
        robot.vuforia.startLook(Vuforia.TargetType.NONE_JUST_RUN_FOREVER);
    }

    @Override
    protected void update() {
        driveUsingInput();
        updateIntake();
        updateArm();
        updateTelemetry();
    }

    private void updateIntake() {
        double intakePower = gamepad1.left_trigger - gamepad1.right_trigger * 0.4;
        robot.intake.left.setPower(intakePower);
        robot.intake.right.setPower(intakePower);
    }

    private void updateArm() {
        double liftPower = (gamepad1.dpad_up ? 1.0 : 0) + (gamepad1.dpad_down? -1.0 : 0);
        double swivelPower = (gamepad1.dpad_left ? -1.0 : 0) + (gamepad1.dpad_right ? 1.0 : 0);

        robot.arm.swivelMotor.setPower(swivelPower);
        robot.arm.swivelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.arm.liftMotor.setPower(-liftPower);
        robot.arm.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void updateTelemetry() {
        if (robot.vuforia.found()) {
            telemetry.addData("Vuforia", robot.vuforia.getPose().toString());
        } else {
            telemetry.addData("Vuforia", "No targets visible");
        }

        telemetry.update();
    }

}
