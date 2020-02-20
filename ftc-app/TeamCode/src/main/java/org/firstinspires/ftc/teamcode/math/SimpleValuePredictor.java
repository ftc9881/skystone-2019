package org.firstinspires.ftc.teamcode.math;

import org.firstinspires.ftc.teamcode.teleop.utility.Command;

import java.util.LinkedList;

public class SimpleValuePredictor {
    private int size;
    private LinkedList<Double> values = new LinkedList<>();
    private LinkedList<Long> times = new LinkedList<>();
    private double maxDelta = 0.0;
    private double additionalDeltaPerSecond = 0.0;

    public SimpleValuePredictor(int size) {
        this.size = Math.max(2, size);
    }

    public SimpleValuePredictor(Command config, String key) {
        this.size = config.getInt(key + " predictor size", 2);
        this.maxDelta = config.getDouble(key + " predictor near delta", 0.0);
        this.additionalDeltaPerSecond = config.getDouble(key + " predictor delta per second", 0.0);
    }

    public void add(Double d) {
        values.addFirst(d);
        times.addFirst(System.currentTimeMillis());
        while (values.size() > size) {
            values.removeLast();
            times.removeLast();
        }
    }

    public boolean canMakePrediction() {
        return values.size() > 1 && times.size() > 1;
    }

    public boolean isNear(double actual) {
        if (canMakePrediction()) {
            double deltaThreshold = maxDelta + additionalDeltaPerSecond * getElapsedTimeSinceFirst();
            double predicted = getPredictedDouble();
            return ( actual > predicted - deltaThreshold ) && ( actual < predicted + deltaThreshold );
        }
        return false;
    }

    public double getPredictedDouble() {
        if (!canMakePrediction()) {
            return 0.0;
        }
        long time = getDeltaTime();
        if (time <= 0) {
            return values.getFirst();
        }
        double newestDouble = values.getFirst();
        double deltaPerMs = getDelta() * (1.0 / getDeltaTime());
        return newestDouble + deltaPerMs * getElapsedTimeSinceFirst();
    }

    private double getDelta() {return values.getFirst()- values.getLast();}

    private long getDeltaTime() {
        return times.getFirst() - times.getLast();
    }

    private long getElapsedTimeSinceFirst() {
        return System.currentTimeMillis() - times.getFirst();
    }

}
