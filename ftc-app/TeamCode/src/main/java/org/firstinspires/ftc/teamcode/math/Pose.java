package org.firstinspires.ftc.teamcode.math;

import android.support.annotation.NonNull;

public class Pose {

    public double x;
    public double y;
    public double r;

    public Pose(double x, double y, double r) {
        this.x = x;
        this.y = y;
        this.r = r;
    }

    public Pose() {
        this(0, 0, 0);
    }

    public Pose(Pose p) {
        this(p.x, p.y, p.r);
    }

    public boolean isAllZero() {
        return this.x == 0 && this.y == 0 && this.r == 0;
    }

    public Pose add(Pose other) {
        return new Pose(x + other.x, y + other.y, r + other.r);
    }

    public Pose subtract(Pose other) {
        return new Pose(x - other.x, y - other.y, r - other.r);
    }

    public Pose multiply(double factor) {
        return new Pose(x * factor, y * factor, r * factor);
    }

    public double distanceTo(Pose b) {
        return Math.sqrt(Math.pow(x-b.x, 2) + Math.pow(y-b.y, 2));
    }

    public boolean sameAs(Pose other) {
        return x == other.x && y == other.y && r == other.r;
    }

    public boolean isWithin(Pose delta, Pose other) {
        return GeneralMath.isWithin(x, delta.x, other.x) && GeneralMath.isWithin(y, delta.y, other.y) && GeneralMath.isWithin(r, delta.r, other.r);
    }

    @NonNull
    public String toString() {
        return String.format("\t%s\t%s\t%s", GeneralMath.round(x, 3), GeneralMath.round(y, 3), GeneralMath.round(r, 3));
    }

    public String toString(String separator) {
        return GeneralMath.round(x, 3) + separator + GeneralMath.round(y, 3) + separator + GeneralMath.round(r, 3);
    }


}
