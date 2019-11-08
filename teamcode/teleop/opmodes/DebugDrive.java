package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.utility.Button;

@TeleOp(name = "Debug Drive", group = "Debug")
public class DebugDrive extends SimpleDrive {

    private Button slowButton;

    @Override
    protected void initialize() {
        super.initialize();
        slowButton = new Button();
    }

    @Override
    protected void update() {
        robot.odometrySystem.updatePose();
        slowButton.update(gamepad1.a);
        drivePowerFactor = slowButton.is(Button.State.HELD) ? 0.5 : 1.0;
        driveUsingInput();
        updateTelemetry();
    }

    private void updateTelemetry() {
        telemetry.addData("Power", driveY);
        telemetry.addData("Power Factor", drivePowerFactor);

        telemetry.addData("LF Position", robot.driveTrain.lf.getCurrentPosition());
        telemetry.addData("RF Position", robot.driveTrain.rf.getCurrentPosition());
        telemetry.addData("LB Position", robot.driveTrain.lb.getCurrentPosition());
        telemetry.addData("RB Position", robot.driveTrain.rb.getCurrentPosition());

        telemetry.addData("Right Encoder", robot.odometrySystem.getRightPosition());
        telemetry.addData("Left Encoder", robot.odometrySystem.getLeftPosition());
        telemetry.addData("Center Encoder", robot.odometrySystem.getCenterPosition());
        telemetry.addData("Odometry Pose", robot.odometrySystem.getPose().toString());

        telemetry.update();
    }
}
