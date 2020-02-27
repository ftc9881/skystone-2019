package org.firstinspires.ftc.teamcode.teleop.opmodes.test;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.teleop.opmodes.drive.BaseDrive;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp(group="Test")
//@Disabled
public class VelocityTest extends BaseDrive {

    private BatMobile batMobile;

    @Override
    protected void initialize() {
        batMobile = BatMobile.createInstance();
        robot.driveTrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        batMobile.elevator.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        batMobile.elevator.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    protected void update() {
        super.update();

        updateElevator();

        telemetry.addData("Left Clicks", batMobile.elevator.left.getCurrentPosition());
        telemetry.addData("RightClicks", batMobile.elevator.right.getCurrentPosition());
    }

    private void updateElevator() {
        batMobile.elevator.setPowerLE(-gamepad2.left_stick_y, gamepad2.right_stick_x);
    }


}