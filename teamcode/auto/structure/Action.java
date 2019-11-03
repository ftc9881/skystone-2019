package org.firstinspires.ftc.teamcode.auto.structure;

public abstract class Action {
    protected boolean isStopped=false;
    protected boolean isDone=false;

    public abstract void run();

    public void stop() {
        isStopped = true;
    }

    public boolean isDone() {
        return isDone;
    }
}
