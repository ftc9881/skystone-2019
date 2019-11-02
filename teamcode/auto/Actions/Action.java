package org.firstinspires.ftc.teamcode.auto.Actions;

public abstract class Action {
    boolean isStopped=false;
    boolean isDone=false;

    public abstract void run();

    public void stop() {
        isStopped = true;
    }

    public boolean isDone() {
        return isDone;
    }
}
