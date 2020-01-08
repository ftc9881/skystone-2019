package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.ArmBot;
import org.firstinspires.ftc.teamcode.robot.BatMobile;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;

@TeleOp
public class BatMobileDrive extends BaseDrive {

    private BatMobile batMobile;

    private double deadZone;
    private double slowDrivePowerFactor;
    private double outtakePowerFactor;

    @Override
    protected void initialize() {
        super.initialize();
        batMobile = BatMobile.newInstance(this);

        deadZone = config.getDouble("dead zone", 0.1);
        slowDrivePowerFactor = config.getDouble("slow drive", 0.4);
        outtakePowerFactor = config.getDouble("outtake power", 0.4);
    }

    @Override
    protected void update() {
        updateDrivePower();
        updateDrive();

        updateIntake();

        updateTelemetry();
    }

    private void updateDrivePower() {
        drivePowerFactor = gamepad1.left_bumper || gamepad1.right_bumper ? slowDrivePowerFactor : 1.0;
    }

    private void updateIntake() {
        double intakePower = (gamepad1.right_trigger - gamepad1.left_trigger) * outtakePowerFactor;
        batMobile.intake.left.setPower(intakePower);
        batMobile.intake.right.setPower(intakePower);
    }

    private void updateTelemetry() {
        telemetry.addData("Drive Power Factor", drivePowerFactor);
        telemetry.addData("Intake Power Factor", outtakePowerFactor);
        telemetry.update();
    }

}
