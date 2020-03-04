package org.firstinspires.ftc.teamcode.teleop.opmodes.test;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teleop.opmodes.drive.BaseDrive;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;

import static org.firstinspires.ftc.teamcode.teleop.utility.Button.State.DOWN;

@TeleOp(group="Test")
//@Disabled
public class VelocityDriveTest extends BaseDrive {

    Button button = new Button();

    @Override
    protected void initialize() {
        robot.driveTrain.setVelocityPIDF();
        robot.driveTrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    protected void update() {
        button.update(gamepad1.b);

        if (button.is(DOWN)) {
            DcMotor.RunMode mode = robot.driveTrain.getMode() == DcMotor.RunMode.RUN_USING_ENCODER ? DcMotor.RunMode.RUN_WITHOUT_ENCODER : DcMotor.RunMode.RUN_USING_ENCODER;
            robot.driveTrain.setMode(mode);
        }

        drivePowerFactor = gamepad1.left_bumper || gamepad1.right_bumper ? 0.5 : 1;
        updateDrive();

        telemetry.addData("Mode", robot.driveTrain.getMode() );
        telemetry.addData("Target Velocity", robot.driveTrain.getMaxTargetVelocity() );
        telemetry.addData("LF Velocity", robot.driveTrain.lf.getVelocity() );
        telemetry.addData("RF Velocity", robot.driveTrain.rf.getVelocity() );
        telemetry.addData("LB Velocity", robot.driveTrain.lb.getVelocity() );
        telemetry.addData("RB Velocity", robot.driveTrain.rb.getVelocity() );
        telemetry.update();
    }

}