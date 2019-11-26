/*
 * Michael - Sharp IR-LED Distance Sensor Test
 */

package org.firstinspires.ftc.teamcode.teleop.opmodes;

import org.firstinspires.ftc.teamcode.sensors.MaxSonarI2CXL;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.sensors.SharpDistanceSensor;

@TeleOp
public class SensorTest extends TeleOpBase {
    private SharpDistanceSensor sharpDistanceSensor;
    private MaxSonarI2CXL sonarTest;

    @Override
    protected void initialize() {
        sharpDistanceSensor = robot.hardwareMap.get(SharpDistanceSensor.class, "sharp");

        sonarTest = robot.hardwareMap.get(MaxSonarI2CXL.class, "sonar");
        sonarTest.startAutoPing(100);
    }

    @Override
    protected void update() {
        telemetry.addData("IR-LED Sensor Distance: ", sharpDistanceSensor.getDistance());

        telemetry.addData("Sonar Connection Info", sonarTest.getConnectionInfo());
        telemetry.addData("Sonar Distance: ", sonarTest.getDistance());

        telemetry.update();
    }

}
