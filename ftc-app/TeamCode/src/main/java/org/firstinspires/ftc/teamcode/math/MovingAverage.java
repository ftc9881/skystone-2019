package org.firstinspires.ftc.teamcode.math;

import org.firstinspires.ftc.teamcode.teleop.utility.Command;

import java.util.LinkedList;

public class MovingAverage {
    private int maxSize;
    private LinkedList<Number> numbers = new LinkedList<>();

    public MovingAverage(Command config, String name) {
        this.maxSize = config.getInt(name + " max size", 1);
    }

    public MovingAverage(int numberDetections) {
        this.maxSize = numberDetections;
    }

    public void add(double value) {
        numbers.push(value);
        while (numbers.size() > maxSize) {
            numbers.removeLast();
        }
    }

    public double getAverage() {
        return GeneralMath.mean(numbers);
    }

}
