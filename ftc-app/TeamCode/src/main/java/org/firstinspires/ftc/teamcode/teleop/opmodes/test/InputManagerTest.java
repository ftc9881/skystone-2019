package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;
import org.firstinspires.ftc.teamcode.teleop.utility.experimental.Axis;
import org.firstinspires.ftc.teamcode.teleop.utility.experimental.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.experimental.InputManager;
import org.firstinspires.ftc.teamcode.teleop.utility.experimental.InputManager.Player;
import org.firstinspires.ftc.teamcode.teleop.utility.experimental.Trigger;

public class InputManagerTest extends TeleOpBase {

    InputManager inputManager;

    @Override
    protected void initialize() {

        inputManager = new InputManager(robot.opMode);
        inputManager
                .addButton("slow", Player.ONE, Button.Input.LEFT_BUMPER)
                .addAxis("drive x", Player.BOTH, Axis.Input.LEFT_STICK_X)
                .addAxis("drive y", Player.BOTH, Axis.Input.LEFT_STICK_Y)
                .addAxis("intake", Player.BOTH, Trigger.Input.LEFT_TRIGGER, Trigger.Input.RIGHT_TRIGGER);
    }

    @Override
    protected void update() {
        telemetry.addData("slow", inputManager.getButton("slow").isPressed());
        telemetry.addData("drive x", inputManager.getAxisValue("drive x"));
        telemetry.addData("drive y", inputManager.getAxisValue("drive y"));
        telemetry.addData("intake", inputManager.getAxisValue("intake"));
    }

}
