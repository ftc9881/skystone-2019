package org.firstinspires.ftc.teamcode.auto.structure;

public abstract class Watcher implements IEndCondition {

    public abstract void update();

    @Override
    public boolean isTrue() {
        update();
        return false;
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {

    }
}
