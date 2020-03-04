package org.firstinspires.ftc.teamcode.auto.endconditions;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Watcher;
import org.firstinspires.ftc.teamcode.hardware.servo.ToggleServo;

public abstract class DeployByDistance extends Watcher {

    private IWatchableDistance watchable;
    private boolean deployed = false;
    private double targetDistance;
    private double currentDistance;
    protected abstract void deploy();

    protected DeployByDistance(IWatchableDistance watchable, double targetDistance) {
        this.watchable = watchable;
        this.targetDistance = targetDistance;
        this.currentDistance = watchable.getDistance();
    }

    public void update() {
        double previousValue = currentDistance;
        currentDistance = watchable.getDistance();
        if (!deployed && (previousValue <= targetDistance && targetDistance <= currentDistance || currentDistance <= targetDistance && targetDistance <= previousValue)) {
            deploy();
            deployed = true;
        }
    }

    @Override
    public void stop() {
        super.stop();
        if (!deployed) {
            deploy();
        }
    }

}
