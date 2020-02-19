package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.servo.CachingServo;
import org.firstinspires.ftc.teamcode.hardware.servo.ToggleServo;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

import java.util.List;

@TeleOp(group="Test")
@Disabled
public class ServosAllTest extends TeleOpBase {

    private Button upButton = new Button();
    private Button downButton = new Button();
    private Button yButton = new Button();
    private Button aButton = new Button();
    private Button lbButton = new Button();
    private Button rbButton = new Button();

    private Button openButton = new Button();
    private Button restButton = new Button();
    private Button closedButton = new Button();

    private List<Servo> servos;
    private int index = 0;
    private CachingServo currentServo;
    private ToggleServo currentToggleServo;

    @Override
    protected void initialize() {
        robot.driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        servos = hardwareMap.getAll(Servo.class);
        if (servos.size() > 0) {
            setCurrentServo(0);
        }
    }

    @Override
    protected void update() {
        if (currentServo != null) {
            updateWhichServo();
            updateConfigureServos();
            updateToggleServo();
            telemetry.update();
        } else {
            telemetry.addData("Uh oh", "No servos found");
            telemetry.update();
        }
    }

    private void updateConfigureServos() {
        upButton.update(gamepad1.dpad_up);
        downButton.update(gamepad1.dpad_down);
        aButton.update(gamepad1.a);
        yButton.update(gamepad1.y);

        if (upButton.is(Button.State.DOWN)) {
            currentServo.setPosition( currentServo.getPosition() + 0.1);
        }
        if (downButton.is(Button.State.DOWN)) {
           currentServo.setPosition( currentServo.getPosition() - 0.1);
        }
        if (yButton.is(Button.State.DOWN)) {
           currentServo.setPosition(currentServo.getPosition() + 0.01);
        }
        if (aButton.is(Button.State.DOWN)) {
           currentServo.setPosition(currentServo.getPosition() - 0.01);
        }
        telemetry.addData(currentServo.getName() ,currentServo.getPosition());
    }

    private void updateToggleServo() {
        openButton.update(gamepad2.a);
        restButton.update(gamepad2.x);
        closedButton.update(gamepad2.y);

        if (openButton.is(Button.State.DOWN)) {
            currentToggleServo.set(ToggleServo.State.OPEN);
        }
        if (restButton.is(Button.State.DOWN)) {
            currentToggleServo.set(ToggleServo.State.REST);
        }
        if (closedButton.is(Button.State.DOWN)) {
            currentToggleServo.set(ToggleServo.State.CLOSED);
        }
        telemetry.addData("ToggleState", currentToggleServo.getState());
    }

    private void updateWhichServo() {
        lbButton.update(gamepad1.left_bumper);
        rbButton.update(gamepad1.right_bumper);

        if (lbButton.is(Button.State.DOWN)) {
            index -= 1;
            wrapIndex();
            setCurrentServo(index);
        }
        if (rbButton.is(Button.State.DOWN)) {
            index += 1;
            wrapIndex();
            setCurrentServo(index);
        }
        telemetry.addData("Index", index);
    }

    private void wrapIndex() {
        if (index < 0) {
            index = servos.size() - 1;
        } else if (index > servos.size() - 1) {
            index = 0;
        }
    }

    private void setCurrentServo(int index) {
        currentServo = new CachingServo(servos.get(index));
        currentToggleServo = new ToggleServo(currentServo);
    }

}
