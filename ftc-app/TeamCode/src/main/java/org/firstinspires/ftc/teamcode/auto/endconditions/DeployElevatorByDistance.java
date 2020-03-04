package org.firstinspires.ftc.teamcode.auto.endconditions;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Watcher;
import org.firstinspires.ftc.teamcode.hardware.servo.ToggleServo;
import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;

public class DeployElevatorByDistance extends DeployByDistance {

    private double liftPower;

    public DeployElevatorByDistance(double liftPower, IWatchableDistance watchable, double targetValue) {
        super(watchable, targetValue);
        this.liftPower = liftPower;
    }

    protected void deploy() {
        BatMobile.getInstance().elevator.setPowerLE(liftPower, 0);
        AutoRunner.log("Watcher", "Deployed lift: " + liftPower);
    }

}
