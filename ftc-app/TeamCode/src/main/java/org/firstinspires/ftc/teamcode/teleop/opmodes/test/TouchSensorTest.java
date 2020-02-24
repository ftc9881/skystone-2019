package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.teleop.opmodes.drive.BaseDrive;

@TeleOp(group="Test")
@Disabled
public class TouchSensorTest extends BaseDrive {

    private TouchSensor touchSensor;

    @Override
    protected void initialize() {
        super.initialize();
        touchSensor = hardwareMap.touchSensor.get("pivot feedback");
    }

    @Override
    protected void update() {
        super.update();
        telemetry.addData("pressed", touchSensor.isPressed());
        telemetry.update();
    }

}
