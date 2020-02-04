package org.firstinspires.ftc.teamcode.auto.structure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.concurrent.atomic.AtomicBoolean;

public abstract class Action implements Runnable {

    private static final long SLEEP_INTERVAL = 10;

    private Thread thread;
    private AtomicBoolean running = new AtomicBoolean(false);
    private AtomicBoolean stopped = new AtomicBoolean(true);

    protected String tag = "Action";
    protected LinearOpMode opMode;

    protected abstract void onRun();
    protected abstract void insideRun() throws SomethingBadHappened;
    protected abstract void onEndRun();
    protected abstract boolean runIsComplete();

    public void start() {
        internalStart();
        thread = new Thread(this);
        thread.start();
    }

    @Override
    public synchronized void run() {
        onRun();

        while (opModeIsActive() && isRunning() && !runIsComplete()) {

            try {
                insideRun();
            } catch (SomethingBadHappened x) {
                opMode.requestOpModeStop();
            }

            try {
                Thread.sleep(SLEEP_INTERVAL);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                AutoRunner.log(tag, "Action thread was interrupted");
                break;
            }

        }

        internalStop();
//        notify();
        AutoRunner.log(tag, "Completed");
    }

    public void stop() {
        AutoRunner.log(tag, "Stop");
        thread.interrupt();
    }

    private void internalStart() {
        running.set(true);
        stopped.set(false);
    }

    private void internalStop() {
        running.set(false);
        stopped.set(true);
        onEndRun();
    }

    public boolean isRunning() {
        return running.get();
    }

    public boolean isStopped() {
        return stopped.get();
    }

    private boolean opModeIsActive() {
        if (opMode == null) {
            opMode = Robot.getInstance().opMode;
        }
        return !opMode.isStopRequested();
    }

    public void runSynchronized(IEndCondition condition) {
        onRun();
        condition.start();
        AutoRunner.log("opmodeactive", opModeIsActive());
        AutoRunner.log("runComplete", runIsComplete());
        AutoRunner.log("condition", condition.isTrue());
        while (opModeIsActive() && !runIsComplete() && !condition.isTrue()) {
            try {
                insideRun();
            } catch (SomethingBadHappened x) {
                opMode.requestOpModeStop();
            }
        }
        condition.stop();
        onEndRun();
        AutoRunner.log(tag, "Completed");
    }

}
