package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.teamcode.auto.endConditions.ObstacleDetect;
import org.firstinspires.ftc.teamcode.auto.structure.IEndCondition;
import org.firstinspires.ftc.teamcode.sensors.MaxSonarI2CXL;

public class SensorSystem{

    private MaxSonarI2CXL sonar;
//    HardwareDevice stoneSwitch;

    public SensorSystem(HardwareMap hardwareMap) {
//        sonar = hardwareMap.get(MaxSonarI2CXL.class, "sonar");
//        sonar.setI2cAddress(I2cAddr.create8bit(0xE0));
//        sonarSensor.startAutoPing(100);
    }

    public boolean stoneIsIn() {
        // TODO: get from switch sensor thingy
        return false;
    }

    public boolean obstacleInFront() {
        // TODO: use sonar
//        return robot.sonarSensor.getDistance() < distance;
        return false;
    }

}
