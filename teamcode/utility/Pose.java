package org.firstinspires.ftc.teamcode.utility;

public class Pose {

    public double x;
    public double y;
    public double r;

    public Pose(double x, double y, double r) {
        this.x = x;
        this.y = y;
        this.r = r;
    }

    public Pose(Pose p) {
        this(p.x, p.y, p.r);
    }

    public Pose add(Pose other) {
        return new Pose(x + other.x, y + other.y, y + other.r);
    }

    public Pose subtract(Pose other) {
        return new Pose(x - other.x, y - other.y, y - other.r);
    }

    public double distanceTo(Pose b) {
        return Math.sqrt(Math.pow(x-b.x, 2) + Math.pow(y-b.y, 2));
    }

    public String toString() {
       return (round(x, 3) + ", " + round(y, 3) + ", " + round(r, 3));
    }

    private double round(double number, int places) {
        double scale = Math.pow(10, places);
        return Math.round(number * scale) / scale;
    }

}
