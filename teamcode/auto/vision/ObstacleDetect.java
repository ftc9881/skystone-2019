package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.Robot;

public class ObstacleDetect {

    private boolean looking = false;
    private DetectRunnable detectRunnable;
    private Robot robot;

    public ObstacleDetect() {

    }

    class DetectRunnable implements Runnable {
        boolean needWait;

        @Override
        public void run() {

            // listen to sensor
            robot.sonarSensor.getDistance();

        }
    }

    public void start() {
        looking = true;
        detectRunnable = new DetectRunnable();
        Thread t = new Thread(detectRunnable);
        t.start();
    }

    public void stop() {
        looking = false;
        try {
            while (detectRunnable.needWait) {
                detectRunnable.wait();
            }
        } catch (InterruptedException e) {
            robot.log("Thread was interrupted in ObstacleDetect", true);
        }
    }

}
