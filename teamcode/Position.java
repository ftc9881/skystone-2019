package org.firstinspires.ftc.teamcode;

public class Position {

    public double x;
    public double y;
    public double r;

    public Position(double x, double y, double r) {
        this.x = x;
        this.y = y;
        this.r = r;
    }

    public Position add(Position other) {
        return new Position(x + other.x, y + other.y, y + other.r);
    }

    public String toString() {
       return (x + ", " + y + ", " + r);
    }

}
