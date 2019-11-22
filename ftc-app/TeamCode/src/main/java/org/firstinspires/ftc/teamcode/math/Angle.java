package org.firstinspires.ftc.teamcode.math;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Angle {

    private double value;
    private AngleUnit unit;

    public Angle(double value, AngleUnit unit) {
        this.value = value;
        this.unit = unit;
    }

    public double getRadians() {
        return unit.toRadians(value);
    }

    public double getDegrees() {
        return unit.toDegrees(value);
    }

    public void setRadians(double radians) {
        this.value = unit.fromRadians(radians);
    }

    public void setDegrees(double degrees) {
        this.value = unit.fromDegrees(degrees);
    }
}
