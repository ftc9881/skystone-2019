package org.firstinspires.ftc.teamcode.teleop.opmodes.test;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.teleop.opmodes.drive.BaseDrive;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

import static org.firstinspires.ftc.teamcode.teleop.utility.Button.State.HELD;

@TeleOp(group="Test")
//@Disabled
public class TestVelocityTest extends TeleOpBase {

    private BatMobile batMobile;
    private Button button = new Button();

    private Pose drivePose = new Pose();

    @Override
    protected void initialize() {
        batMobile = BatMobile.createInstance();
        robot.driveTrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveTrain.setVelocityPIDF();
    }

    @Override
    protected void update() {
        button.update(gamepad1.left_bumper || gamepad1.right_bumper);

        drivePose.y = button.is(HELD) ? 0.5 : 0;
        robot.driveTrain.drive(drivePose);
//        updateElevator();

        telemetry.addData("LF Velocity", robot.driveTrain.lf.getVelocity() );
        telemetry.addData("RF Velocity", robot.driveTrain.rf.getVelocity() );
        telemetry.addData("LB Velocity", robot.driveTrain.lb.getVelocity() );
        telemetry.addData("RB Velocity", robot.driveTrain.rb.getVelocity() );
        telemetry.update();
    }


    private void updateElevator() {
        batMobile.elevator.setPowerLE(-gamepad2.left_stick_y, gamepad2.right_stick_x);
    }


}