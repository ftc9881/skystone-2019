package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.devices.ToggleServo;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp
//@Disabled
public class ServoTest extends TeleOpBase {

    private Button upButton = new Button();
    private Button downButton = new Button();
    private Button yButton = new Button();
    private Button aButton = new Button();
    private Button bButton = new Button();

    private String servoName;
    private Servo servo;

    private ToggleServo toggleServo;

    @Override
    protected void initialize() {
        robot.driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        servoName = config.getString("debug servo", "NONE");
        if (!servoName.equals("NONE")) {
            servo = hardwareMap.servo.get(servoName);
            toggleServo = new ToggleServo(hardwareMap, servoName);
        }
    }

    @Override
    protected void update() {
        if (servo != null) {
            updateConfigureServos();
            updateToggleServo();
            telemetry.update();
        }
    }

    private void updateConfigureServos() {
        upButton.update(gamepad1.dpad_up);
        downButton.update(gamepad1.dpad_down);
        aButton.update(gamepad1.a);
        yButton.update(gamepad1.y);

        if (upButton.is(Button.State.DOWN)) {
            servo.setPosition( servo.getPosition() + 0.1);
        }
        if (downButton.is(Button.State.DOWN)) {
            servo.setPosition( servo.getPosition() - 0.1);
        }
        if (yButton.is(Button.State.DOWN)) {
            servo.setPosition( servo.getPosition() + 0.01);
        }
        if (aButton.is(Button.State.DOWN)) {
            servo.setPosition( servo.getPosition() - 0.01);
        }
        telemetry.addData(servoName, servo.getPosition());
    }

    private void updateToggleServo() {
        bButton.update(gamepad1.b);
        if (bButton.is(Button.State.DOWN)) {
            toggleServo.toggle(ToggleServo.State.CLOSED, ToggleServo.State.REST);
        }
        telemetry.addData("ToggleState", toggleServo.getState());
    }

}
