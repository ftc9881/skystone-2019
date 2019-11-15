/*
 * Quick test of limit switch
 * NOTE: May have to add hardware map stuff?
 */

package org.firstinspires.ftc.teamcode.teleop.opmodes;

import org.firstinspires.ftc.teamcode.teleop.TeleOpBase;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class LimitSwitchTest extends TeleOpBase {
    private TouchSensor blockLimitSwitch;

    @Override
    protected void initialize() {
        blockLimitSwitch = new Button();
    }

    @Override
    protected void update() {
        if (blockLimitSwitch.isPressed())
            telemetry.addData("Limit switch pressed.");
    }

}