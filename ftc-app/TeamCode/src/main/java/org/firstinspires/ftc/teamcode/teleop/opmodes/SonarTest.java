/*
 * Michael - Sonar Sensor Test
 *
 */

package org.firstinspires.ftc.teamcode.teleop.opmodes;

import org.firstinspires.ftc.teamcode.teleop.TeleOpBase;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.sensors.MaxSonarI2CXL;
import org.firstinspires.ftc.teamcode.sensors.SonarArrayManager; // not using this rn

@TeleOp
public class SonarTest extends TeleOpBase {
    private MaxSonarI2CXL sonarTest;

    @Override
    protected void initialize() {
        // Ex: usage from Limit Switch test
        // ---
        // blockLimitSwitch = robot.hardwareMap.touchSensor.get("limit");

        sonarTest = robot.hardwareMap.get(MaxSonarI2CXL.class, "sonar");
        sonarTest.startAutoPing(100);
    }

    @Override
    protected void update() {
        telemetry.addData("Test Sensor: getConnectionInfo()", sonarTest.getConnectionInfo());
        telemetry.addData("Sonar Distance: ", sonarTest.getDistance());
        telemetry.update();
    }

}