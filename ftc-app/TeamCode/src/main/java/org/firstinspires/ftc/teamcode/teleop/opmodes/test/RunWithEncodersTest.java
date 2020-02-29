package org.firstinspires.ftc.teamcode.teleop.opmodes.test;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.teleop.opmodes.drive.BaseDrive;

@TeleOp(group="Test")
//@Disabled
public class RunWithEncodersTest extends BaseDrive {

    @Override
    protected void initialize() {
        super.initialize();
        robot.driveTrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    protected void update() {
        super.update();
        telemetry.addData("LB Velocity", robot.driveTrain.lb.getVelocity());
    }
}