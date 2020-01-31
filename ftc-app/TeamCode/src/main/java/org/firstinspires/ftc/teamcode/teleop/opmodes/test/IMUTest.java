package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.structure.SomethingBadHappened;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp
public class IMUTest extends TeleOpBase {

    @Override
    protected void initialize() {
    }

    @Override
    protected void update() {
        telemetry.addData("imu", robot.imu.getHeading().getDegrees());
        telemetry.update();
    }

}
