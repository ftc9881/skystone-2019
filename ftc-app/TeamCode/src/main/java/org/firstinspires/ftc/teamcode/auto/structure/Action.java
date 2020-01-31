package org.firstinspires.ftc.teamcode.auto.structure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.concurrent.atomic.AtomicBoolean;

import static android.os.SystemClock.sleep;

public abstract class Action implements Runnable {

    private LinearOpMode opMode;

    private static final String TAG = "Action";
    private static final long SLEEP_INTERVAL = 10;
    private Thread thread;
    private AtomicBoolean running = new AtomicBoolean(false);
    private AtomicBoolean stopped = new AtomicBoolean(true);

    protected abstract void onRun();
    protected abstract void insideRun() throws SomethingBadHappened;
    protected abstract void onEndRun();
    protected abstract boolean runIsComplete();

    public void start() {
        AutoRunner.log(TAG, "Start");
        running.set(true);
        stopped.set(false);

        AutoRunner.log("Action running?", isRunning());

        thread = new Thread(this);
        thread.start();
    }

    @Override
    public synchronized void run() {
        onRun();

        while (isRunning() && !runIsComplete() && opModeIsActive()) {
            try {
                Thread.sleep(SLEEP_INTERVAL);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                AutoRunner.log(TAG, "Action thread was interrupted");
                return;
            }

            try {
                insideRun();
            } catch (SomethingBadHappened x) {
                Robot.getInstance().opMode.requestOpModeStop();
            }
        }

        onEndRun();
        stopped.set(true);
        notify();

        AutoRunner.log(TAG, "Completed");
    }

    public void stop() {
        AutoRunner.log(TAG, "Stop");

        if (!isStopped())
            onEndRun();
        running.set(false);
        stopped.set(true);

        thread.interrupt();
    }

    public boolean isRunning() {
        return running.get();
    }

    public boolean isStopped() {
        return stopped.get();
    }

    private boolean opModeIsActive() {
        if (opMode == null) {
            Robot robot = Robot.getInstance();
            if (robot != null) {
                opMode = robot.opMode;
            }
        }
        return opMode != null && opMode.opModeIsActive();
    }

}
