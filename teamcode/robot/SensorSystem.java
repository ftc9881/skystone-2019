package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.teamcode.sensors.MaxSonarI2CXL;

public class SensorSystem implements IRobotSystem {

    MaxSonarI2CXL sonar;

    public SensorSystem(HardwareMap hardwareMap) {

//        sonar = hardwareMap.get(MaxSonarI2CXL.class, "sonar");
//        sonar.setI2cAddress(I2cAddr.create8bit(0xE0));
//        sonarSensor.startAutoPing(100);

    }

}
