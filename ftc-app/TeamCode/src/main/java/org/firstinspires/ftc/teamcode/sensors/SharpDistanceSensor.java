package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by ftc on 11/19/2017.
 */

public class SharpDistanceSensor implements IDistanceSensor {
    private AnalogInput _analogInput;

    private double _a = 0.069663618;
    private double _b = 0.001511491;
    private double _c = 1.653738727;

    public SharpDistanceSensor(HardwareMap hardwareMap, String name) {
        _analogInput = hardwareMap.analogInput.get(name);
    }

    public SharpDistanceSensor(HardwareMap hardwareMap, String name, double a, double b, double c) {
        _analogInput = hardwareMap.analogInput.get(name);
        _a = a;
        _b = b;
        _c = c;
    }

    public double getVoltage() {
        return _analogInput.getVoltage();
    }

    public double getDistance() {
        double voltReading = getVoltage();

        if(_a * voltReading - _b > 0) {
            return (1 / (_a * voltReading - _b)) - _c;
        }
        else {
            return Double.NaN;
        }
    }
}
