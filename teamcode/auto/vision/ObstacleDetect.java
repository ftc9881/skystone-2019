package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.Robot;

public class ObstacleDetect {

    private boolean looking = false;
    private DetectRunnable detectRunnable;
    private Robot robot;

    public ObstacleDetect() {

    }

    class DetectRunnable implements Runnable {
        @Override
        public void run() {

            // listen to sensor
            robot.sonarSensor.getDistance();

        }
    }

    public void startDetect() {
        looking = true;
        detectRunnable = new DetectRunnable();
        Thread t = new Thread(detectRunnable);
        t.start();
    }

//    public void stopLook() {
//        looking = false;
//        try {
//            while (lookRunnable.needWait) {
//                lookRunnable.wait();
//            }
//        } catch (InterruptedException e) {
//            robot.log("VUFORIA", "Thread was interrupted", true);
//        }
//    }
//
//    public boolean found() {
//        return lastPose != null;
//    }
//
//    public Pose getPose() {
//        return lastPose;
//    }
//
}
