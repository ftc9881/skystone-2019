package org.firstinspires.ftc.teamcode.auto.structure;

import com.qualcomm.robotcore.util.RobotLog;
import java.util.concurrent.atomic.AtomicBoolean;

public abstract class Action implements Runnable {

    private static final long SLEEP_INTERVAL = 50;
    private Thread thread;
    private AtomicBoolean running = new AtomicBoolean(false);
    private AtomicBoolean stopped = new AtomicBoolean(true);

    protected abstract void onRun();
    protected abstract void insideRun();
    protected abstract void onEndRun();
    protected abstract boolean runIsComplete();

    public void start() {
        RobotLog.d("Action: Start");
        running.set(true);
        stopped.set(false);

        thread = new Thread(this);
        thread.start();
    }

    @Override
    public void run() {
        onRun();

        while (isRunning() && !runIsComplete()) {
            try {
                Thread.sleep(SLEEP_INTERVAL);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                RobotLog.d("Action thread was interrupted");
                return;
            }
            insideRun();
        }

        onEndRun();
        stopped.set(true);
        RobotLog.d("Action: Completed");
    }


    public void stop() {
        RobotLog.d("Action: Stop");

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

}
