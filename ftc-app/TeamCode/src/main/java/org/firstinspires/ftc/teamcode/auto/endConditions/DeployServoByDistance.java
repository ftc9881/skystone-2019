package org.firstinspires.ftc.teamcode.auto.endConditions;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Watcher;
import org.firstinspires.ftc.teamcode.robot.devices.ToggleServo;

public class DeployServoByDistance extends Watcher {

    private ToggleServo toggleServo;
    private DcMotor trackingMotor;
    private int clicksToDeployAt;
    private boolean deployed = false;

    public DeployServoByDistance(ToggleServo toggleServo, DcMotor trackingMotor, int clicksToDeployAt) {
        this.toggleServo = toggleServo;
        this.trackingMotor = trackingMotor;
        this.clicksToDeployAt = Math.abs(clicksToDeployAt);
    }

    public void update() {
        if (!deployed && clicksToDeployAt - Math.abs(trackingMotor.getCurrentPosition()) < 0) {
            toggleServo.toggle();
            AutoRunner.log("DEPLOYED SERVO");
            deployed = true;
        }

    }

}
