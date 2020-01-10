/*
 * Michael - Sharp IR-LED Distance Sensor Test
 */

package org.firstinspires.ftc.teamcode.teleop.opmodes;

import org.firstinspires.ftc.teamcode.sensors.MaxSonarI2CXL;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.sensors.SharpDistanceSensor;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp
@Disabled
public class SensorTest extends TeleOpBase {
    private SharpDistanceSensor sharpDistanceSensor;
    private MaxSonarI2CXL sonarSensor;
    private DigitalChannel digitalDistanceSensor;

    @Override
    protected void initialize() {

        digitalDistanceSensor = robot.hardwareMap.digitalChannel.get("digital");

//        AnalogInput sharpAnalogInput = robot.hardwareMap.analogInput.get("sharp");
//        sharpDistanceSensor = new SharpDistanceSensor(sharpAnalogInput);

//        sonarSensor = robot.hardwareMap.get(MaxSonarI2CXL.class, "sonar");
//        sonarSensor.startAutoPing(100);

    }

    @Override
    protected void update() {
        telemetry.addData("Digital", digitalDistanceSensor.getState());

//        telemetry.addData("IR-LED Sensor Distance: ", sharpDistanceSensor.getDistance());

//        telemetry.addData("Sonar Connection Info", sonarSensor.getConnectionInfo());
//        telemetry.addData("Sonar Distance: ", sonarSensor.getDistance());

        telemetry.update();
    }

}
