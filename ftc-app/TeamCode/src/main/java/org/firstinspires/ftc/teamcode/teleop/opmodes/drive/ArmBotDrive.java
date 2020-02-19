package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.ArmBot.ArmBot;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;

@TeleOp(group="Drive")
@Disabled
public class ArmBotDrive extends BaseDrive {

    private ArmBot armBot;

    private double slowDrivePowerFactor;
    private double defaultDrivePowerFactor;
    private double fastDrivePowerFactor;
    private double outtakePowerFactor;

    private Button foundationButton = new Button();
    private Button stoneButton = new Button();
    private Button capstoneButton = new Button();

    @Override
    protected void initialize() {
        super.initialize();
        armBot = ArmBot.getInstance();

        slowDrivePowerFactor = config.getDouble("slow drive", 0.4);
        defaultDrivePowerFactor = config.getDouble("default drive", 0.8);
        fastDrivePowerFactor = config.getDouble("fast drive", 1.0);
        outtakePowerFactor = config.getDouble("outtake power", 0.4);
    }

    @Override
    protected void update() {
        updateDrivePower();
        updateDrive();

        updateArm();
        updateServos();
        updateIntake();

        updateTelemetry();
    }

    private void updateArm() {
        double power = (gamepad1.dpad_right ? 1 : 0) - (gamepad1.dpad_left ? 1 : 0);
        armBot.arm.pivotMotor.setPower(power);
    }

    private void updateServos() {
        foundationButton.update(gamepad1.y);
        stoneButton.update(gamepad1.a);
        capstoneButton.update(gamepad1.x);

        if (foundationButton.is(Button.State.DOWN)) {
            armBot.foundationServo.toggle();
        }

        if (stoneButton.is(Button.State.DOWN)) {
            armBot.stoneServo.toggle();
        }

        if (capstoneButton.is(Button.State.DOWN)) {
            armBot.capstoneServo.toggle();
        }
    }

    private void updateDrivePower() {
        if (gamepad1.left_bumper) {
            drivePowerFactor = slowDrivePowerFactor;
        }
        else if (gamepad1.right_bumper) {
            drivePowerFactor = fastDrivePowerFactor;
        }
        else {
            drivePowerFactor = defaultDrivePowerFactor;
        }
    }

    private void updateIntake() {
        double intakePower = (gamepad1.right_trigger - gamepad1.left_trigger) * outtakePowerFactor;
        armBot.intake.left.setPower(intakePower);
        armBot.intake.right.setPower(intakePower);
    }

    private void updateTelemetry() {
        telemetry.addData("Drive Power Factor", drivePowerFactor);
        telemetry.update();
    }

}
