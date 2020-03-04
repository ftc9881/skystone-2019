package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleop.opmodes.drive.BaseDrive;

@TeleOp(group="Test")
//@Disabled
public class RevSensorTest extends BaseDrive {

    private Rev2mDistanceSensor frontSensor;
    private Rev2mDistanceSensor leftSensor;
    private Rev2mDistanceSensor rightSensor;

    @Override
    protected void initialize() {
        super.initialize();
        frontSensor = hardwareMap.get(Rev2mDistanceSensor.class, "front sensor");
        leftSensor = hardwareMap.get(Rev2mDistanceSensor.class, "left side sensor");
        rightSensor = hardwareMap.get(Rev2mDistanceSensor.class, "right side sensor");

    }

    @Override
    protected void update() {
        super.update();
        telemetry.addData("front", frontSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("left", leftSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("right", rightSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }

}
