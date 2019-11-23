/*
 * Michael - Sharp IR-LED Distance Sensor Test
 */

package org.firstinspires.ftc.teamcode.teleop.opmodes;

import org.firstinspires.ftc.teamcode.teleop.TeleOpBase;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.sensors.SharpDistanceSensor;

@TeleOp
public class SharpDistanceSensorTest extends TeleOpBase {
    private SharpDistanceSensor sharpTest;

    @Override
    protected void initialize() {
        sharpTest = new SharpDistanceSensor(robot.hardwareMap.analogInput.get("sharp"));
    }

    @Override
    protected void update() {
        telemetry.addData("IR-LED Sensor Distance: ", sharpTest.getDistance());
        telemetry.update();
    }

}
