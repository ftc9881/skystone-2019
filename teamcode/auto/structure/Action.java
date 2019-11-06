package org.firstinspires.ftc.teamcode.auto.structure;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;

import java.util.concurrent.atomic.AtomicBoolean;

public abstract class Action implements Runnable {

    private static final String TAG = "Action";
    private static final long SLEEP_INTERVAL = 50;
    private Thread thread;
    private AtomicBoolean running = new AtomicBoolean(false);
    private AtomicBoolean stopped = new AtomicBoolean(true);

    protected abstract void onRun();
    protected abstract void insideRun();
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
    public void run() {
        onRun();

        while (isRunning() && !runIsComplete()) {
            try {
                Thread.sleep(SLEEP_INTERVAL);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                AutoRunner.log(TAG, "Action thread was interrupted");
                return;
            }
            insideRun();
        }

        onEndRun();
        stopped.set(true);
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

}
