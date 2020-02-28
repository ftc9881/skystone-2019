package org.firstinspires.ftc.teamcode.auto.endconditions;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Watcher;
import org.firstinspires.ftc.teamcode.hardware.servo.ToggleServo;
import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;

public class DeployElevatorByDistance extends Watcher {

    private double liftPower;
    private IWatchableDistance watchable;
    private double targetValue;
    private boolean deployed = false;
    private double currentValue;

    public DeployElevatorByDistance(double liftPower, IWatchableDistance watchable, double targetValue) {
        this.liftPower = liftPower;
        this.watchable = watchable;
        this.targetValue = targetValue;
        this.currentValue = watchable.getDistance();
    }

    private void deploy() {
        BatMobile.getInstance().elevator.setPowerLE(liftPower, 0);
        AutoRunner.log("Watcher", "Deployed lift: " + liftPower);
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
