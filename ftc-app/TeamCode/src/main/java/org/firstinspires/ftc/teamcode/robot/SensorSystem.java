package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.sensors.MaxSonarI2CXL;

public class SensorSystem{

    private MaxSonarI2CXL sonar;
    TouchSensor stoneSwitch;

    public SensorSystem(HardwareMap hardwareMap) {
//        sonar = hardwareMap.get(MaxSonarI2CXL.class, "sonar");
//        sonar.setI2cAddress(I2cAddr.create8bit(0xE0));
//        sonarSensor.startAutoPing(100);
//        stoneSwitch = hardwareMap.touchSensor.get("limit");
    }

    public boolean stoneIsIn() {
        return stoneSwitch.isPressed();
    }

    public boolean obstacleInFront() {
        // TODO: use sonar
//        return robot.sonarSensor.getDistance() < distance;
        return false;
    }

}
