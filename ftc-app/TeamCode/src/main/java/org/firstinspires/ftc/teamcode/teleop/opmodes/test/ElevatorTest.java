package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.robot.BatMobile.DifferentialElevator;
import org.firstinspires.ftc.teamcode.teleop.opmodes.drive.BaseDrive;

@TeleOp(group="Test")
public class ElevatorTest extends BaseDrive {

    DifferentialElevator elevator;

    @Override
    protected void initialize() {
        super.initialize();
        elevator = BatMobile.getInstance().elevator;
    }

    @Override
    protected void update() {
        updateElevatorManual();

        telemetry.addData("Left Clicks", elevator.leftLift.motor.getCurrentPosition());
        telemetry.addData("Right Clicks", elevator.rightLift.motor.getCurrentPosition());
        telemetry.update();
    }

    private void updateElevatorManual() {
        double liftPower = -gamepad2.left_stick_y;
        double extendPower = gamepad2.right_stick_x;
        elevator.setPowerLE(liftPower, extendPower);
    }

}
