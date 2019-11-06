package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.utility.Odometry;

@TeleOp(name = "Debug Drive", group = "Debug")
public class DebugDrive extends SimpleDrive {

    private Odometry odometry;
    private Button slowButton;

    @Override
    protected void initialize() {
        super.initialize();
        odometry = new Odometry(robot);
        slowButton = new Button();
    }

    @Override
    protected void update() {
        odometry.updatePose();
        slowButton.update(gamepad1.a);
        drivePowerFactor = slowButton.is(Button.State.HELD) ? 0.5 : 1.0;
        driveUsingInput();
        updateTelemetry();
    }

    private void updateTelemetry() {
        telemetry.addData("Power", driveY);
        telemetry.addData("Power Factor", drivePowerFactor);

        telemetry.addData("Right Encoder", robot.getOdometryRightPosition());
        telemetry.addData("Left Encoder", robot.getOdometryLeftPosition());
        telemetry.addData("Center Encoder", robot.getOdometryCenterPosition());
        telemetry.addData("Odometry Pose", odometry.getPose().toString());

        telemetry.update();
    }
}
