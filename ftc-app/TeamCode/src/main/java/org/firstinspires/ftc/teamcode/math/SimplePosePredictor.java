package org.firstinspires.ftc.teamcode.math;

import org.firstinspires.ftc.teamcode.teleop.utility.Command;

import java.util.LinkedList;

public class SimplePosePredictor {
    private int size;
    private LinkedList<Pose> poses = new LinkedList<>();
    private LinkedList<Long> times = new LinkedList<>();
    private Pose maxDelta = new Pose();
    private Pose additionalDeltaPerSecond = new Pose();

    public SimplePosePredictor(int size) {
        this.size = Math.max(2, size);
    }

    public SimplePosePredictor(Command config, String key) {
        this.size = config.getInt(key + " pose predictor size", 2);
        this.maxDelta = config.getPose(key + " pose predictor near delta");
        this.additionalDeltaPerSecond = config.getPose(key + " pose predictor delta per second");
    }

    public void add(Pose pose) {
        poses.add(pose);
        times.add(System.currentTimeMillis());
        while (poses.size() > size) {
            poses.removeLast();
            times.removeLast();
        }
    }

    public boolean canMakePrediction() {
        return poses.size() > 1 && times.size() > 1;
    }

    public boolean predictionIsNearActual(Pose actualPose) {
        if (canMakePrediction()) {
            Pose deltaRange = maxDelta.add( additionalDeltaPerSecond.multiply(getElapsedTimeSinceFirst()) );
            return getPredictedPose().isWithin(deltaRange, actualPose);
        }
        return false;
    }

    public Pose getPredictedPose() {
        if (!canMakePrediction()) {
            return new Pose();
        }
        long time = getDeltaTime();
        if (time <= 0) {
            return poses.getFirst();
        }
        Pose newestPose = poses.getFirst();
        Pose deltaPerMs = getDeltaPose().multiply(1.0/getDeltaTime());
        return newestPose.add( deltaPerMs.multiply(getElapsedTimeSinceFirst()) );
    }

    private Pose getDeltaPose() {
        return poses.getFirst().subtract(poses.getLast());
    }

    private long getDeltaTime() {
        return times.getFirst() - times.getLast();
    }

    private long getElapsedTimeSinceFirst() {
        return System.currentTimeMillis() - times.getFirst();
    }

}
