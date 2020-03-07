package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp(group="Test")
//@Disabled
public class ArmsTest extends TeleOpBase {

    private Button toggleRedClaw = new Button();
    private Button toggleBlueClaw = new Button();
    private Button toggleRedPivot = new Button();
    private Button toggleBluePivot = new Button();

    private BatMobile batMobile;

    @Override
    protected void initialize() {
        robot.driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        batMobile = BatMobile.createInstance();
    }

    @Override
    protected void update() {
        toggleRedClaw.update(gamepad1.y);
        toggleRedPivot.update(gamepad1.b);
        toggleBlueClaw.update(gamepad1.dpad_up);
        toggleBluePivot.update(gamepad1.dpad_left);

        if (toggleRedClaw.is(Button.State.DOWN)) {
            batMobile.redSideArm.claw.toggle();
        }
        if (toggleBlueClaw.is(Button.State.DOWN)) {
            batMobile.blueSideArm.claw.toggle();
        }
        if (toggleRedPivot.is(Button.State.DOWN)) {
            batMobile.redSideArm.pivot.toggle();
        }
        if (toggleBluePivot.is(Button.State.DOWN)) {
            batMobile.blueSideArm.pivot.toggle();
        }

        telemetry.addData("Red Pivot-", batMobile.redSideArm.pivot.getState());
        telemetry.addData("Red Claw--", batMobile.redSideArm.claw.getState());
        telemetry.addData("Blue Pivot", batMobile.blueSideArm.pivot.getState());
        telemetry.addData("Blue Claw-", batMobile.blueSideArm.claw.getState());
        telemetry.update();
    }

}
