package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;

@TeleOp(name = "Debug Drive", group = "Debug")
//@Disabled
public class DebugDrive extends BaseDrive {

    private Button upButton = new Button();
    private Button downButton = new Button();

    private String servoName;
    private Servo servo;

    @Override
    protected void initialize() {
        super.initialize();

        robot.initializeImu(AngleUnit.DEGREES);

        servoName = config.getString("debugServo", "servo");
        servo = hardwareMap.servo.get(servoName);
    }

    @Override
    protected void update() {
        super.update();

        updateConfigureServos();
        updateTelemetry();
    }

    private void updateConfigureServos() {
        upButton.update(gamepad1.dpad_up);
        downButton.update(gamepad1.dpad_down);

        if (upButton.is(Button.State.DOWN)) {
            servo.setPosition( servo.getPosition() - 0.1);
        }
        if (downButton.is(Button.State.DOWN)) {
            servo.setPosition( servo.getPosition() + 0.1);
        }

        telemetry.addData(servoName, servo.getPosition());
    }


    private void updateTelemetry() {
        telemetry.addData("LF", robot.driveTrain.lf.getCurrentPosition());
        telemetry.addData("LB", robot.driveTrain.lb.getCurrentPosition());
        telemetry.addData("RF", robot.driveTrain.rf.getCurrentPosition());
        telemetry.addData("RB", robot.driveTrain.rb.getCurrentPosition());

        telemetry.addData("firstAngle", robot.imu.getAngularOrientation().firstAngle);
        telemetry.addData("secondAngle", robot.imu.getAngularOrientation().secondAngle);
        telemetry.addData("thirdAngle", robot.imu.getAngularOrientation().thirdAngle);

        telemetry.update();
    }

}
