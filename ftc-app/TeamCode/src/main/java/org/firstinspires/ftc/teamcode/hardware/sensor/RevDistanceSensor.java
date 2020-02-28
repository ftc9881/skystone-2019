package org.firstinspires.ftc.teamcode.hardware.sensor;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RevDistanceSensor implements IDistanceSensor {
    private Rev2mDistanceSensor delegate;

    public RevDistanceSensor(HardwareMap hardwareMap, String name) {
        this.delegate = hardwareMap.get(Rev2mDistanceSensor.class, name);
    }

    public RevDistanceSensor(Rev2mDistanceSensor delegate) {
        this.delegate = delegate;
    }

    @Override
    public double getDistance() {
        return delegate.getDistance(DistanceUnit.INCH);
    }
}
