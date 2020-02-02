package org.firstinspires.ftc.teamcode.teleop.utility.experimental;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;

public class GamepadCombiner {

    private ArrayList<Gamepad> gamepads;

    public boolean a;

    public GamepadCombiner() {
        gamepads = new ArrayList<>();
    }

    public GamepadCombiner add(Gamepad gamepad) {
        gamepads.add(gamepad);
        return this;
    }

    public Gamepad getCombinedGamepadOutput() {
        Gamepad result = new Gamepad();
        for (Gamepad gamepad : gamepads) {
            result.a = result.a || gamepad.a;
            result.b = result.b || gamepad.b;
            result.x = result.x || gamepad.x;
            result.y = result.y || gamepad.y;

            result.dpad_down = result.dpad_down || gamepad.dpad_down;
            result.dpad_up = result.dpad_up || gamepad.dpad_up;
            result.dpad_left = result.dpad_left || gamepad.dpad_left;
            result.dpad_right = result.dpad_right || gamepad.dpad_right;

            result.left_bumper = result.left_bumper || gamepad.left_bumper;
            result.right_bumper = result.right_bumper || gamepad.right_bumper;

            result.left_stick_button = result.left_stick_button || gamepad.left_stick_button;
            result.right_stick_button = result.right_stick_button || gamepad.right_stick_button;

            result.right_trigger = gamepad.right_trigger > result.right_trigger ? gamepad.right_trigger : result.right_trigger;
            result.left_trigger = gamepad.left_trigger > result.left_trigger ? gamepad.left_trigger : result.left_trigger;

            result.right_stick_x = Math.abs(gamepad.right_stick_x) > Math.abs(result.right_stick_x) ? gamepad.right_stick_x : result.right_stick_x;
            result.right_stick_y = Math.abs(gamepad.right_stick_y) > Math.abs(result.right_stick_y) ? gamepad.right_stick_y : result.right_stick_y;
            result.left_stick_x = Math.abs(gamepad.left_stick_x) > Math.abs(result.left_stick_x) ? gamepad.left_stick_x : result.left_stick_x;
            result.left_stick_y = Math.abs(gamepad.left_stick_y) > Math.abs(result.left_stick_y) ? gamepad.left_stick_y : result.left_stick_y;
        }
        return result;
    }

}
