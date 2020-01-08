package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.robot.Elevator;

@Autonomous
public class TestElevatorAutoOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();

        Elevator elevator = new Elevator(hardwareMap);

        while (opModeIsActive()) {

            elevator.updateVelocity();
            elevator.leftLift.motor.setTargetPosition(1000);
            elevator.rightLift.motor.setTargetPosition(-1000);
            elevator.leftLift.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator.rightLift.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator.leftLift.motor.setPower(1);
            elevator.rightLift.motor.setPower(1);

            telemetry.addData("left lift encoder", elevator.leftLift.motor.getCurrentPosition());
            telemetry.addData("right lift encoder", elevator.rightLift.motor.getCurrentPosition());

            telemetry.addData("left lift velocity", elevator.leftLift.getVelocity());
            telemetry.addData("right lift velocity", elevator.rightLift.getVelocity());

            telemetry.update();
        }
    }

}
