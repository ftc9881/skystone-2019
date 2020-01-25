package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp
//@Disabled
public class ServoTest extends TeleOpBase {

    private Button upButton = new Button();
    private Button downButton = new Button();
    private Button yButton = new Button();
    private Button aButton = new Button();

    private String servoName;
    private Servo servo;

    @Override
    protected void initialize() {
        robot.driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        servoName = config.getString("debug servo", "NONE");
        if (!servoName.equals("NONE")) {
            servo = hardwareMap.servo.get(servoName);
        }
    }

    @Override
    protected void update() {
        if (servo != null) {
            updateConfigureServos();
        }
        updateTelemetry();
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


    private void updateTelemetry() {
        telemetry.addData("LF", robot.driveTrain.lf.getCurrentPosition());
        telemetry.addData("LB", robot.driveTrain.lb.getCurrentPosition());
        telemetry.addData("RF", robot.driveTrain.rf.getCurrentPosition());
        telemetry.addData("RB", robot.driveTrain.rb.getCurrentPosition());

        telemetry.update();
    }

}
