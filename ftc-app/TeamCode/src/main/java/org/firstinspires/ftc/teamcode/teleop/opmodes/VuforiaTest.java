package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp
//@Disabled
public class VuforiaTest extends TeleOpBase {

    Vuforia vuforia;

    @Override
    protected void initialize() {
        vuforia = new Vuforia(hardwareMap);
        vuforia.initialize();
        vuforia.startLook(VisionSystem.TargetType.RUN_FOREVER);
    }

    @Override
    protected void update() {
        Pose pose = vuforia.getPose();
        telemetry.addData("X (in)", pose.x);
        telemetry.addData("Y (in)", pose.y);
        telemetry.addData("R (deg)", pose.r);
       telemetry.update();
    }

}
