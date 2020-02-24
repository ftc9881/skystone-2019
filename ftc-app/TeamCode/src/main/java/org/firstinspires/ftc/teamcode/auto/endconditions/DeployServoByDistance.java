package org.firstinspires.ftc.teamcode.auto.endconditions;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Watcher;
import org.firstinspires.ftc.teamcode.hardware.servo.ToggleServo;

public class DeployServoByDistance extends Watcher {

    private ToggleServo toggleServo;
    private DcMotor trackingMotor;
    private int clicksToDeployAt;
    private boolean deployed = false;
    private ToggleServo.State state;
    private int startClicks;

    public DeployServoByDistance(ToggleServo toggleServo, ToggleServo.State state, DcMotor trackingMotor, int clicksToDeployAt) {
        this.toggleServo = toggleServo;
        this.state = state;
        this.trackingMotor = trackingMotor;
        this.clicksToDeployAt = Math.abs(clicksToDeployAt);
        this.startClicks = trackingMotor.getCurrentPosition();
    }

    private void deploy() {
        toggleServo.set(state);
        AutoRunner.log("Watcher", "Deployed servo: " + toggleServo.servo.getName());
        deployed = true;
    }

    public void update() {
        if (!deployed && Math.abs(trackingMotor.getCurrentPosition() - startClicks) > clicksToDeployAt) {
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
