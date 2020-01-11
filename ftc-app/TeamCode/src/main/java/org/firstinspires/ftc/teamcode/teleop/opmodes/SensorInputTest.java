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
import org.firstinspires.ftc.teamcode.sensors.SharpPair;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp
@Disabled
public class SensorInputTest extends TeleOpBase {
    private SharpPair detector;

    @Override
    protected void initialize() {

        AnalogInput sensor_left_analog = robot.hardwareMap.analogInput.get("bdL");
        AnalogInput sensor_right_analog = robot.hardwareMap.analogInput.get("bdR");
        SharpDistanceSensor sensor_left = new SharpDistanceSensor(sensor_left_analog);
        SharpDistanceSensor sensor_right = new SharpDistanceSensor(sensor_right_analog);

        detector = new SharpPair(sensor_left, sensor_right,
                config.getDouble("blockDetectDist", 21),
                config.getDouble("blockDetectMargin", 2));
    }

    @Override
    protected void update() {
        // telemetry.addData("Block Detected:", detector.blockDetected());
        telemetry.addData("== STATE ==", detector.getState());
        telemetry.addData("== BLOCK DETECTED ==", detector.blockDetected());

//      telemetry.addData("Digital", digitalDistanceSensor.getState());
//      telemetry.addData("IR-LED Sensor Distance: ", sharpDistanceSensor.getDistance());
//      telemetry.adddata("sonar connection info", sonarsensor.getconnectioninfo());
//      telemetry.adddata("sonar distance: ", sonarsensor.getdistance());

        telemetry.update();
    }

}
