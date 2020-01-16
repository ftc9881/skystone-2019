package org.firstinspires.ftc.teamcode.auto.endconditions;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Watcher;
import org.firstinspires.ftc.teamcode.robot.devices.ToggleServo;

public class DeployServoByDistance extends Watcher {

    private ToggleServo toggleServo;
    private DcMotor trackingMotor;
    private int clicksToDeployAt;
    private boolean deployed = false;
    private ToggleServo.State state = ToggleServo.State.REST;

    public DeployServoByDistance(ToggleServo toggleServo, ToggleServo.State state, DcMotor trackingMotor, int clicksToDeployAt) {
        this.toggleServo = toggleServo;
        this.state = state;
        this.trackingMotor = trackingMotor;
        this.clicksToDeployAt = Math.abs(clicksToDeployAt);
    }

    public void update() {
        if (!deployed && clicksToDeployAt - Math.abs(trackingMotor.getCurrentPosition()) < 0) {
            toggleServo.set(state);
            AutoRunner.log("DEPLOYED SERVO");
            deployed = true;
        }

    }

}
