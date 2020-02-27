package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.hardware.motor.OdometryWheel;
import org.firstinspires.ftc.teamcode.hardware.sensor.MaxSonarAnalogSensor;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.teleop.opmodes.drive.BaseDrive;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;

@TeleOp(group="Test")
//@Disabled
public class OdometryTest extends BaseDrive {

    private OdometryWheel odometryY;

    private Button toggleFloatButton = new Button();

    @Override
    protected void initialize() {
        super.initialize();
        odometryY = BatMobile.getInstance().odometryY;
        odometryY.resetEncoder();
    }

    @Override
    protected void update() {
        super.update();

        toggleFloatButton.update(gamepad1.a);

        telemetry.addData("odometry y (clicks)", odometryY.getClicks());
        telemetry.addData("odometry y (in)", odometryY.getInches());

        telemetry.update();
    }

}
