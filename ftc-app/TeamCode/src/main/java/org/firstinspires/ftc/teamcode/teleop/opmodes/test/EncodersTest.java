package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.teleop.opmodes.drive.BaseDrive;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp(group="Test")
//@Disabled
public class EncodersTest extends BaseDrive {

    @Override
    protected void initialize() {
        super.initialize();
        robot.driveTrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    protected void update() {
        super.update();
        telemetry.addData("lf", robot.driveTrain.lf.getCurrentPosition());
        telemetry.addData("rf", robot.driveTrain.rf.getCurrentPosition());
        telemetry.addData("lb", robot.driveTrain.lb.getCurrentPosition());
        telemetry.addData("rb", robot.driveTrain.rb.getCurrentPosition());
        telemetry.update();
    }

}
