/*
 * Quick test of limit switch
 * NOTE: May have to add hardware map stuff?
 */

package org.firstinspires.ftc.teamcode.teleop.opmodes;

import org.firstinspires.ftc.teamcode.teleop.TeleOpBase;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class LimitSwitchTest extends TeleOpBase {
    private TouchSensor blockLimitSwitch;

    @Override
    protected void initialize() {
        blockLimitSwitch = robot.hardwareMap.touchSensor.get("limit");
    }

    @Override
    protected void update() {
        telemetry.addData("Limit Switch", blockLimitSwitch.isPressed());
        telemetry.update();
    }

}