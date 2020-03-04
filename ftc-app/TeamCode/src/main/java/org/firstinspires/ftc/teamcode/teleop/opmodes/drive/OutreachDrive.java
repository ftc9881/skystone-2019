package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group="Drive")
@Disabled
public class OutreachDrive extends BaseDrive {

    private double slowDrivePowerFactor;
    private double defaultDrivePowerFactor;
    private double fastDrivePowerFactor;

    @Override
    protected void initialize() {
        super.initialize();

        slowDrivePowerFactor = config.getDouble("slow drive", 0.4);
        defaultDrivePowerFactor = config.getDouble("default drive", 0.8);
        fastDrivePowerFactor = config.getDouble("fast drive", 1.0);
    }

    @Override
    protected void update() {
        updateDrivePower();
        updateDrive();

        updateTelemetry();
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

    private void updateTelemetry() {
        telemetry.addData("Drive Power Factor", drivePowerFactor);
        telemetry.update();
    }

}
