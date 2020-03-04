package org.firstinspires.ftc.teamcode.auto.endconditions;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Watcher;
import org.firstinspires.ftc.teamcode.hardware.servo.ToggleServo;

public class DeployServoByDistance extends DeployByDistance {

    private ToggleServo toggleServo;
    private ToggleServo.State state;

    public DeployServoByDistance(ToggleServo toggleServo, ToggleServo.State state, IWatchableDistance watchable, double targetValue) {
        super(watchable, targetValue);
        this.toggleServo = toggleServo;
        this.state = state;
    }

    protected void deploy() {
        toggleServo.set(state);
        AutoRunner.log("Watcher", "Deployed servo: " + toggleServo.servo.getName());
    }

}
