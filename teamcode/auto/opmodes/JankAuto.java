package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name = "Jank", group = "TeamCode")
public class JankAuto extends LinearOpMode {

    private static final int LIFT_UP_POSITION = 0;
    private static final int LIFT_DOWN_POSITION = 1000;

    private static final int SWIVEL_IN_POSITION = 0;
    private static final int SWIVEL_OUT_POSITION = 7000;

    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this);

        robot.arm.swivelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        robot.drive(0, 1 ,0);
        sleep(750);

        robot.stop();

        autoArmMoveOut();
   }


    private void autoArmMoveOut() {

        int swivelTargetPosition = SWIVEL_OUT_POSITION;
        robot.arm.swivelMotor.setTargetPosition(swivelTargetPosition);
        robot.arm.swivelMotor.setPower(0.5);
        robot.arm.swivelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.arm.liftMotor.setPower(-0.5);

        while (robot.arm.swivelMotor.isBusy()) {
            sleep(20);
        }

        robot.arm.liftMotor.setPower(0);
    }

}

