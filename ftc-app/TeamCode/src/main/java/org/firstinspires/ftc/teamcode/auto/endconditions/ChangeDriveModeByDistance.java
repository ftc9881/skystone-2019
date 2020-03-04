package org.firstinspires.ftc.teamcode.auto.endconditions;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.hardware.servo.ToggleServo;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class ChangeDriveModeByDistance extends DeployByDistance {

    private DcMotor.RunMode mode;

    public ChangeDriveModeByDistance(IWatchableDistance watchable, double targetValue, DcMotor.RunMode mode) {
        super(watchable, targetValue);
        this.mode = mode;
    }

    protected void deploy() {
        Robot.getInstance().driveTrain.setMode(mode);
        AutoRunner.log("Watcher", "Changed mode: " + mode.name());
    }

}
