package org.firstinspires.ftc.teamcode.math;

import org.opencv.core.Point;

public class Line {

    public Point point1;
    public Point point2;

    public Line() {
        point1 = new Point();
        point2 = new Point();
    }

    public Line(double[] coordinates) {
        point1 = new Point(coordinates[0], coordinates[1]);
        point2 = new Point(coordinates[2], coordinates[3]);
    }

    public Line(Point point1, Point point2) {
        this.point1 = point1;
        this.point2 = point2;
    }

    public double length() {
        return GeneralMath.distance(point1, point2);
    }

    public double width() {
        return Math.abs(point2.x - point1.x);
    }

    public double height() {
        return Math.abs(point2.y - point1.y);
    }

    public Point center() {
        return new Point((int) (0.5*(point1.x + point2.x)),(int) (0.5*(point1.y + point2.y)));
    }

    public double angle() {
        return Math.atan2(point2.y-point1.y,point2.x-point1.x);
    }

    public void resize(double scale) {
        this.point1 = new Point(point1.x * scale,point1.y * scale);
        this.point2 = new Point(point2.x * scale,point2.y * scale);
    }

    public double minX() {
        return Math.min(point1.x, point2.x);
    }

    public double maxX() {
        return Math.max(point1.x, point2.x);
    }

    public double minY() {
        return Math.min(point1.y, point2.y);
    }

    public double maxY() {
        return Math.max(point1.y, point2.y);
    }

    @Override
    public String toString() {
        return "{" + point1.x + "," + point1.y + "} to {" + point2.x + "," + point2.y + "}";
    }

}
