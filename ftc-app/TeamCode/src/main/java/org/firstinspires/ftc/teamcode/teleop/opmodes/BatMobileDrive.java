package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.robot.devices.ToggleServo;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class BatMobileDrive extends BaseDrive {

    private BatMobile batMobile;

    private double liftPowerFactor;
    private double extendPowerFactor;
    private double slowDrivePowerFactor;
    private double outtakePowerFactor;

    private Button pivotButton = new Button();
    private Button clawButton = new Button();
    private Button foundationButton = new Button();

    @Override
    protected void initialize() {
        super.initialize();
//        batMobile = BatMobile.newInstance(this);
        batMobile = BatMobile.getInstance();

        liftPowerFactor = config.getDouble("lift power", 1.0);
        extendPowerFactor = config.getDouble("extend power", 1.0);
        slowDrivePowerFactor = config.getDouble("slow drive", 0.4);
        outtakePowerFactor = config.getDouble("outtake power", 0.4);
    }

    @Override
    protected void update() {
        updateDrivePower();
        updateDrive();

        updateElevator();
        updateIntake();
        updateServos();

        updateTelemetry();
    }

    private void updateDrivePower() {
        drivePowerFactor = gamepad1.left_bumper || gamepad1.right_bumper ? slowDrivePowerFactor : 1.0;
    }

    private void updateElevator() {
        double liftPower = (gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0);
        double extendPower = (gamepad1.dpad_right ? 1 : 0) - (gamepad1.dpad_left ? 1 : 0);
        batMobile.elevator.lift.setPower(liftPower * liftPowerFactor);
        batMobile.elevator.extend.setPower(extendPower * extendPowerFactor);
    }

    private void updateIntake() {
        double intakePower = (gamepad1.right_trigger - gamepad1.left_trigger) * outtakePowerFactor;
        batMobile.intake.left.setPower(intakePower);
        batMobile.intake.right.setPower(intakePower);
    }

    private void updateServos() {
        updateToggle(pivotButton, gamepad1.y, batMobile.sideArm.pivot);
        updateToggle(clawButton, gamepad1.x, batMobile.sideArm.claw);
        List<ToggleServo> foundationServos = new ArrayList<>();
        foundationServos.add(batMobile.leftFoundationServo);
        foundationServos.add(batMobile.rightFoundationServo);
        updateToggle(foundationButton, gamepad1.b, foundationServos);
    }

    private void updateToggle(Button button, boolean input, ToggleServo servo) {
        button.update(input);
        if (button.is(Button.State.DOWN)) {
            servo.toggle();
        }
    }

    private void updateToggle(Button button, boolean input, List<ToggleServo> servos) {
        button.update(input);
        if (button.is(Button.State.DOWN)) {
            for (ToggleServo servo : servos) {
                servo.toggle();
            }
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Drive Power Factor", drivePowerFactor);
        telemetry.addData("Intake Power Factor", outtakePowerFactor);
        telemetry.update();
    }

}
