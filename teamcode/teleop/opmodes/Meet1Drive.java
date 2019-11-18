package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.Arm;

@TeleOp(name = "Meet 1 Drive", group = "TeleOp")
public class Meet1Drive extends BaseDrive {

    private double deadZone;
    private double intakePower;
    private double outtakePower;

    private int swivelOuterLimit = Arm.State.OUT.swivelPosition;
    private int swivelInnerLimit = Arm.State.IN.swivelPosition;

    @Override
    protected void initialize() {
        super.initialize();

        deadZone = config.getDouble("dead zone", 0.1);
        intakePower = config.getDouble("intake power", 0.4);
        outtakePower = config.getDouble("outtake power", 0.4);
    }

    @Override
    protected void update() {
        super.update();

        updateArmLimitsFromInput(gamepad2);
        moveArmFromInput(gamepad2);
        intakeFromInput(bothGamepads);

        updateTelemetry();
    }

    private void intakeFromInput(Gamepad gamepad) {
        double presetPower = (gamepad.left_bumper ? intakePower : 0) - (gamepad.right_bumper ? outtakePower : 0);
        double customPower = gamepad.left_trigger - gamepad.right_trigger;
        double intakePower = presetPower + customPower;
        robot.intake.left.setPower(intakePower);
        robot.intake.right.setPower(intakePower);
    }

    private void updateArmLimitsFromInput(Gamepad gamepad) {
        if (gamepad.dpad_right) {
            swivelOuterLimit = robot.arm.swivelMotor.getCurrentPosition();
        } else if (gamepad.dpad_left) {
            swivelInnerLimit = robot.arm.swivelMotor.getCurrentPosition();
        }
    }

    private void moveArmFromInput(Gamepad gamepad) {
        moveSwivel(gamepad);

        if (isGivingLiftInput(gamepad)) {
            moveLift(gamepad);
        } else {
            holdLiftPosition();
        }
    }

    private void moveSwivel(Gamepad gamepad) {
        int swivelPosition = robot.arm.swivelMotor.getCurrentPosition();
        double swivelPower = gamepad.right_stick_x;

        if (swivelPosition > swivelOuterLimit)  {
            swivelPower = Math.min(swivelPower, 0);
        }
        if (swivelPosition < swivelInnerLimit) {
            swivelPower = Math.max(swivelPower, 0);
        }

        robot.arm.swivelMotor.setPower(swivelPower);
    }

    private boolean isGivingLiftInput(Gamepad gamepad) {
        return Math.abs(gamepad.left_stick_y) > deadZone;
    }

    private void moveLift(Gamepad gamepad) {
        if (robot.arm.liftMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            robot.arm.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

         double liftPower = -gamepad.left_stick_y;
        robot.arm.liftMotor.setPower(liftPower);
    }

    private void holdLiftPosition() {
        if (robot.arm.liftMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            int currentPosition = robot.arm.liftMotor.getCurrentPosition();
            robot.arm.liftMotor.setTargetPosition(currentPosition);
            robot.arm.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Lift Position", robot.arm.liftMotor.getCurrentPosition());
        telemetry.addData("Swivel Position", robot.arm.swivelMotor.getCurrentPosition());
        telemetry.addData("Swivel Outer Limit", swivelOuterLimit);
        telemetry.addData("Swivel Inner Limit", swivelInnerLimit);
        telemetry.update();
    }

}
