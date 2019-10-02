package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Created by ftc on 11/19/2017.
 */

public class SharpDistanceSensor implements DistanceSensorIF {
    private AnalogInput _analogInput;

    private double _a = 0.069663618;
    private double _b = 0.001511491;
    private double _c = 1.653738727;

    public SharpDistanceSensor(AnalogInput analogInput) {
        _analogInput = analogInput;
    }

    public SharpDistanceSensor(AnalogInput analogInput, double a, double b, double c) {
        _analogInput = analogInput;

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
