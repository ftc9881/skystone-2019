package org.firstinspires.ftc.teamcode.hardware.sensor;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class SharpSensorPair {

    public SharpDistanceSensor left;
    public SharpDistanceSensor right;

    public SharpSensorPair(SharpDistanceSensor left, SharpDistanceSensor right) {
        this.left = left;
        this.right = right;
    }

    public SharpSensorPair(HardwareMap hardwareMap, String left, String right) {
        this.left = new SharpDistanceSensor(hardwareMap, left);
        this.right = new SharpDistanceSensor(hardwareMap, right);
    }

    public double getDifference() {
        return right.getDistance() - left.getDistance();
    }

    public double getAverage() {
        return (left.getDistance() + right.getDistance()) / 2.0;
    }

}
