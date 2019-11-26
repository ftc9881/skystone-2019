package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.InputManager.Player;

@TeleOp(name = "Debug Drive", group = "Debug")
@Disabled
public class DebugDrive extends BaseDrive {

    @Override
    protected void initialize() {
        super.initialize();
        inputManager.addButton("servo up", Player.BOTH, Button.Input.DPAD_UP);
        inputManager.addButton("servo down", Player.BOTH, Button.Input.DPAD_UP);
    }

    @Override
    protected void update() {
        super.update();

//        updateConfigureServos();
        updateTelemetry();
    }

    private void updateConfigureServos() {
        if (inputManager.buttonJustPressed("servo up")) {
            robot.foundationGrabber.leftServo.setPosition( robot.foundationGrabber.leftServo.getPosition() - 0.1);
            robot.foundationGrabber.rightServo.setPosition( robot.foundationGrabber.rightServo.getPosition() - 0.1);
        }
        if (inputManager.buttonJustPressed("servo down")) {
            robot.foundationGrabber.leftServo.setPosition( robot.foundationGrabber.leftServo.getPosition() + 0.1);
            robot.foundationGrabber.rightServo.setPosition( robot.foundationGrabber.rightServo.getPosition() + 0.1);
        }

        telemetry.addData("Left Grab", robot.foundationGrabber.leftServo.getPosition());
        telemetry.addData("Right Grab", robot.foundationGrabber.rightServo.getPosition());
    }


    private void updateTelemetry() {

        telemetry.addData("LF Position", robot.driveTrain.lf.getCurrentPosition());
        telemetry.addData("RF Position", robot.driveTrain.rf.getCurrentPosition());
        telemetry.addData("LB Position", robot.driveTrain.lb.getCurrentPosition());
        telemetry.addData("RB Position", robot.driveTrain.rb.getCurrentPosition());

        telemetry.addData("Swivel", robot.arm.swivelMotor.getCurrentPosition());
        telemetry.addData("Lift", robot.arm.liftMotor.getCurrentPosition());

        telemetry.addData("IMU Heading", robot.getImuHeading().getDegrees());

//        telemetry.addData("Right Encoder", robot.odometrySystem.getRightPosition());
//        telemetry.addData("Left Encoder", robot.odometrySystem.getLeftPosition());
//        telemetry.addData("Center Encoder", robot.odometrySystem.getCenterPosition());
//        telemetry.addData("Odometry Pose", robot.odometrySystem.getPose().toString());

        telemetry.update();
    }

}
