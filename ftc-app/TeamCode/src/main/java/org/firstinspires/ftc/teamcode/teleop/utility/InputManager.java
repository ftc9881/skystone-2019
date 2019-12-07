package org.firstinspires.ftc.teamcode.teleop.utility;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;

import java.util.HashMap;
import java.util.Map;

public class InputManager {

    public enum Player { ONE, TWO, BOTH }

    private Gamepad gamepad1;
    private Gamepad gamepad2;

    private OpMode opMode;

    private GamepadCombiner gamepadCombiner;
    private Gamepad bothGamepads;

    private Map<String, Button> buttonMap = new HashMap<>();
    private Map<String, Trigger> triggerMap = new HashMap<>();
    private Map<String, Axis> axisMap = new HashMap<>();

    public InputManager(OpMode opMode) {
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;

        this.opMode = opMode;

        gamepadCombiner = new GamepadCombiner();
        gamepadCombiner.add(gamepad1).add(gamepad2);
        bothGamepads = gamepadCombiner.getCombinedGamepadOutput();
    }

    void update() {
        bothGamepads = gamepadCombiner.getCombinedGamepadOutput();
        for (Button button : buttonMap.values()) {
            button.update();
        }
    }

    public InputManager addButton(String name, Player player, Button.Input input) {
        buttonMap.put(name, new Button(getGamepadFor(player), input));
        return this;
    }

    public InputManager addTrigger(String name, Player player, Trigger.Input input) {
        triggerMap.put(name, new Trigger(getGamepadFor(player), input));
        return this;
    }

    public InputManager addAxis(String name, Player player, Axis.Input input) {
        axisMap.put(name, new Axis(getGamepadFor(player), input));
        AutoRunner.log(name, input);
        return this;
    }

    public InputManager addAxis(String name, Player player, Button.Input negativeInput, Button.Input positiveInput) {
        Gamepad gamepad = getGamepadFor(player);
        addAxis(name, new Button(gamepad, negativeInput), new Button(gamepad, positiveInput));
        return this;
    }

    public InputManager addAxis(String name, Player player, Trigger.Input negativeInput, Trigger.Input positiveInput) {
        Gamepad gamepad = getGamepadFor(player);
        addAxis(name, new Trigger(gamepad, negativeInput), new Trigger(gamepad, positiveInput));
        return this;
    }

    public InputManager addAxis(String name, IInput negativeInput, IInput positiveInput) {
        axisMap.put(name, new Axis(negativeInput, positiveInput));
        return this;
    }

    public double getAxisValue(String name) {
        return axisMap.get(name).getValue();
    }

    public double getTriggerValue(String name) {
        return triggerMap.get(name).getValue();
    }

    public Button getButton(String name) {
        return buttonMap.get(name);
    }

    public boolean buttonJustPressed(String name) {
        return getButton(name).is(Button.State.DOWN);
    }


    Gamepad getGamepadFor(Player player) {
        switch (player) {
            default:
            case ONE:
                return gamepad1;
            case TWO:
                return gamepad2;
            case BOTH:
                return bothGamepads;
        }
    }

}
