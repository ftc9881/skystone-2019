package org.firstinspires.ftc.teamcode.hardware.sensor;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;

public class MaxSonarAnalogSensor implements IDistanceSensor {
    private AnalogInput _analogInput;

    private double _m = 315.495;
    private double _b = 1.08555;

    public MaxSonarAnalogSensor(HardwareMap hardwareMap, String name) {
        _analogInput = hardwareMap.analogInput.get(name);
        Configuration config = new Configuration("HardwareConstants");
        _m = config.getDouble(name + " m", _m);
        _b = config.getDouble(name + " b", _b);
    }

    public MaxSonarAnalogSensor(HardwareMap hardwareMap, String name, double m, double b) {
        _analogInput = hardwareMap.analogInput.get(name);
        _m = m;
        _b = b;
    }

    public double getVoltage() {
        return _analogInput.getVoltage();
    }

    public double getDistance() {
        double voltReading = getVoltage();
        return _m * voltReading + _b;
    }
}
