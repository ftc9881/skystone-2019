package org.firstinspires.ftc.teamcode.auto.endconditions;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Watcher;
import org.firstinspires.ftc.teamcode.hardware.servo.ToggleServo;

public class DeployServoByDistance extends Watcher {

    private ToggleServo toggleServo;
    private IWatchableDistance watchable;
    private double targetValue;
    private boolean deployed = false;
    private ToggleServo.State state;
    private double currentValue;

    public DeployServoByDistance(ToggleServo toggleServo, ToggleServo.State state, IWatchableDistance watchable, double targetValue) {
        this.toggleServo = toggleServo;
        this.state = state;
        this.watchable = watchable;
        this.targetValue = targetValue;
        this.currentValue = watchable.getDistance();
    }

    private void deploy() {
        toggleServo.set(state);
        AutoRunner.log("Watcher", "Deployed servo: " + toggleServo.servo.getName());
        deployed = true;
    }

    public void update() {
        double previousValue = currentValue;
        currentValue = watchable.getDistance();
        if (!deployed && (previousValue <= targetValue && targetValue <= currentValue || currentValue <= targetValue && targetValue <= previousValue)) {
            deploy();
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
