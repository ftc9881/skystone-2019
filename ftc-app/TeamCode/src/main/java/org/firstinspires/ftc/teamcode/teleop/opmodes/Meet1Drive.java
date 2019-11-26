package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teleop.utility.Axis;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.InputManager.Player;
import org.firstinspires.ftc.teamcode.teleop.utility.Trigger;

@TeleOp(name = "Meet 1 Drive", group = "TeleOp")
public class Meet1Drive extends BaseDrive {

    private double deadZone;
    private double slowDrivePowerFactor;
    private double defaultDrivePowerFactor;
    private double fastDrivePowerFactor;
    private double outtakePowerFactor;

    @Override
    protected void initialize() {
        super.initialize();

        deadZone = config.getDouble("dead zone", 0.1);
        slowDrivePowerFactor = config.getDouble("slow drive", 0.4);
        defaultDrivePowerFactor = config.getDouble("default drive", 0.8);
        fastDrivePowerFactor = config.getDouble("fast drive", 1.0);
        outtakePowerFactor = config.getDouble("outtake power", 0.4);

        inputManager
            .addButton("slow", Player.ONE, Button.Input.LEFT_BUMPER)
            .addButton("fast", Player.ONE, Button.Input.RIGHT_BUMPER)
            .addAxis("p1 lift", Player.ONE, Button.Input.DPAD_DOWN, Button.Input.DPAD_UP)
            .addAxis("p1 swivel", Player.ONE, Button.Input.DPAD_LEFT, Button.Input.DPAD_RIGHT)

            .addAxis("p2 lift", Player.TWO, Axis.Input.LEFT_STICK_Y)
            .addAxis("p2 swivel", Player.TWO, Axis.Input.RIGHT_STICK_X)

            .addAxis("intake", Player.BOTH, Trigger.Input.LEFT_TRIGGER, Trigger.Input.RIGHT_TRIGGER)
            .addButton("grab", Player.BOTH, Button.Input.B);
    }

    @Override
    protected void update() {
        updateDrivePower();
        updateDrive();

        updateFoundationGrabToggle();
        updateSwivelMove();
        updateLiftMove();
        updateIntake();

        updateTelemetry();
    }

    private void updateDrivePower() {
        if (inputManager.getButton("slow").isPressed()) {
            drivePowerFactor = slowDrivePowerFactor;
        }
        else if (inputManager.getButton("fast").isPressed()) {
            drivePowerFactor = fastDrivePowerFactor;
        }
        else {
            drivePowerFactor = defaultDrivePowerFactor;
        }
    }

    private void updateFoundationGrabToggle() {
        if (inputManager.buttonJustPressed("grab")) {
            robot.foundationGrabber.toggle();
        }
    }

    private void updateIntake() {
        double intakePower = inputManager.getAxisValue("intake") * outtakePowerFactor;
        robot.intake.left.setPower(intakePower);
        robot.intake.right.setPower(intakePower);
    }

    private void updateSwivelMove() {
        double power = inputManager.getAxisValue("p1 swivel") + inputManager.getAxisValue("p2 swivel");
        robot.arm.swivelMotor.setPower(power);
    }

    private void updateLiftMove() {
        double liftInput = inputManager.getAxisValue("p1 lift") + inputManager.getAxisValue("p2 lift");
        if (Math.abs(liftInput) > deadZone) {
            moveLift(liftInput);
        } else {
            holdLiftPosition();
        }
    }

    private void moveLift(double liftInput) {
        if (robot.arm.liftMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            robot.arm.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        robot.arm.liftMotor.setPower(liftInput);
    }

    private void holdLiftPosition() {
        if (robot.arm.liftMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            int currentPosition = robot.arm.liftMotor.getCurrentPosition();
            robot.arm.liftMotor.setTargetPosition(currentPosition);
            robot.arm.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Drive Power Factor", drivePowerFactor);
        telemetry.addData("Lift Position", robot.arm.liftMotor.getCurrentPosition());
        telemetry.addData("Swivel Position", robot.arm.swivelMotor.getCurrentPosition());
        telemetry.update();
    }

}
