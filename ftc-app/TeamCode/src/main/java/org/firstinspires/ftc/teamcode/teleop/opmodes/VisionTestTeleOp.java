package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp(name="Vision Test")
public class VisionTestTeleOp extends TeleOpBase {

    @Override
    protected void initialize() {
        robot.visionSystem.initialize();
        robot.visionSystem.startLook(VisionSystem.TargetType.NONE_JUST_RUN_FOREVER);
    }

    @Override
    protected void update() {
        telemetry.update();
    }

}
