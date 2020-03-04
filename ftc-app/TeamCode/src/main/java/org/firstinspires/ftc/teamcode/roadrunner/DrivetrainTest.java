package org.firstinspires.ftc.teamcode.roadrunner;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp(group = "roadrunner")
@Disabled
public class DrivetrainTest extends TeleOpBase {

    @Override
    protected void initialize() {
        robot.driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    protected void update() {
        telemetry.addData("LF", robot.driveTrain.lf.getCurrentPosition());
        telemetry.addData("RF", robot.driveTrain.rf.getCurrentPosition());
        telemetry.addData("LB", robot.driveTrain.lb.getCurrentPosition());
        telemetry.addData("RB", robot.driveTrain.rb.getCurrentPosition());
        telemetry.update();
    }

}
