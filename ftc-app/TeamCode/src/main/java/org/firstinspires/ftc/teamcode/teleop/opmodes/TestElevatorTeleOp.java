package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.devices.VelocityMotor;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp
public class TestElevatorTeleOp extends TeleOpBase {

    private VelocityMotor leftLift;
    private VelocityMotor rightLift;

    private double liftPower;
    private Button downButton = new Button();
    private double leftPower;
    private double rightPower;

    @Override
    protected void initialize() {
        leftLift = new VelocityMotor( hardwareMap.dcMotor.get("left lift"));
        rightLift = new VelocityMotor( hardwareMap.dcMotor.get("right lift"));

        liftPower = config.getDouble("lift power", 0.4);

        leftLift.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    protected void update() {
        leftLift.update();
        rightLift.update();
        downButton.update(gamepad2.dpad_down);

        if (downButton.is(Button.State.DOWN)) {
            leftLift.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLift.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftLift.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightLift.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (gamepad2.a) {
            // Down
            leftPower = -1;
            rightPower = -1;
        } else if (gamepad2.b) {
            // Right
            leftPower = -1;
            rightPower = 1;
        } else if (gamepad2.x) {
            // Left
            leftPower = 1;
            rightPower = -1;
        } else if (gamepad2.y) {
            // Up
            leftPower = 1;
            rightPower = 1;
        } else {
            leftPower = gamepad1.left_stick_y;
            rightPower = gamepad1.right_stick_y;
        }

        leftLift.motor.setPower(leftPower * liftPower);
        rightLift.motor.setPower(rightPower * liftPower);

        telemetry.addData("Player 1", "Lift Control (left/right stick y)");
        telemetry.addData("Player 2", "Lift Control (a/b/x/y)");

        telemetry.addData("left lift encoder", leftLift.motor.getCurrentPosition());
        telemetry.addData("right lift encoder", rightLift.motor.getCurrentPosition());

        telemetry.addData("left lift power", leftPower);
        telemetry.addData("right lift power", rightPower);

        telemetry.addData("left lift velocity", leftLift.getVelocity());
        telemetry.addData("right lift velocity", rightLift.getVelocity());

        telemetry.update();
    }

}
