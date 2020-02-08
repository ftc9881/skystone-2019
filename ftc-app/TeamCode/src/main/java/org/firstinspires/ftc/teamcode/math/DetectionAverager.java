package org.firstinspires.ftc.teamcode.math;

import org.firstinspires.ftc.teamcode.teleop.utility.Command;

import java.util.LinkedList;
import java.util.Queue;

public class DetectionAverager {
    private int numberDetections;
    private LinkedList<Double> detections = new LinkedList<>();
    private int average;
    public DetectionAverager(Command config, String name) {
        this.numberDetections = config.getInt(name + " detection count", 1);
    }
    public void addDetection(double detection) {
        detections.add(detection);
        while (detections.size() > numberDetections) {
            detections.pop();
        }
    }
    public double getAverageDetection() {
        double averagedDetection = 0;
        for (double i : detections)
            averagedDetection += i;
        return averagedDetection / detections.size();
    }
}
