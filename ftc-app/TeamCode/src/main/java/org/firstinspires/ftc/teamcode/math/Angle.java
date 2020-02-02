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
        return unit == AngleUnit.DEGREES ? value * Math.PI / 180 : value;
    }

    public double getDegrees() {
        return unit == AngleUnit.RADIANS ? value / Math.PI * 180 : value;
    }

}
