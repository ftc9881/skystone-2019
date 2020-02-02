package org.firstinspires.ftc.teamcode.auto.structure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.concurrent.atomic.AtomicBoolean;

import static android.os.SystemClock.sleep;

public abstract class Action implements Runnable {

    private static final String TAG = "Action";
    private static final long SLEEP_INTERVAL = 10;

    private Thread thread;
    private AtomicBoolean running = new AtomicBoolean(false);
    private AtomicBoolean stopped = new AtomicBoolean(true);

    protected LinearOpMode opMode;

    protected abstract void onRun();
    protected abstract void insideRun() throws SomethingBadHappened;
    protected abstract void onEndRun();
    protected abstract boolean runIsComplete();

    public void start() {
        running.set(true);
        stopped.set(false);
        thread = new Thread(this);
        thread.start();
    }

    @Override
    public synchronized void run() {
        onRun();

        while (opModeIsActive() && isRunning() && !runIsComplete()) {
            try {
                Thread.sleep(SLEEP_INTERVAL);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                AutoRunner.log(TAG, "Action thread was interrupted");
                internalStop();
                return;
            }

            try {
                insideRun();
            } catch (SomethingBadHappened x) {
                opMode.requestOpModeStop();
            }
        }

        internalStop();
        AutoRunner.log(TAG, "Completed");
    }

    public void stop() {
        AutoRunner.log(TAG, "Stop");
        thread.interrupt();
    }

    private void internalStop() {
        running.set(false);
        stopped.set(true);
        onEndRun();
        notify();
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

}
