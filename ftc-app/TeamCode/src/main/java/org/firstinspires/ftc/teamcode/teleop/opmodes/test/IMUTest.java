package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.structure.SomethingBadHappened;
import org.firstinspires.ftc.teamcode.teleop.opmodes.drive.BaseDrive;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp(group="Test")
@Disabled
public class IMUTest extends BaseDrive {

    @Override
    protected void initialize() {
        super.initialize();
        robot.initializeIMU();
    }

    @Override
    protected void update() {
        super.update();
        telemetry.addData("imu", robot.imu.getHeading().getDegrees());
        telemetry.addData("imu integrated", robot.imu.getIntegratedHeading().getDegrees());
        telemetry.update();
    }

}
