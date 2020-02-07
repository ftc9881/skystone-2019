package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.SomethingBadHappened;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

@TeleOp(group="Test")
@Disabled
public class ExceptionTest extends TeleOpBase {

    Button throwButton = new Button();

    @Override
    protected void initialize() {
    }

    @Override
    protected void update() {
        throwButton.update(gamepad1.a);

        if (throwButton.is(Button.State.DOWN)) {
            try {
                throw new SomethingBadHappened("Did we exit?");
            } catch (SomethingBadHappened exception) {
                requestOpModeStop();
            }
        }
    }

}
