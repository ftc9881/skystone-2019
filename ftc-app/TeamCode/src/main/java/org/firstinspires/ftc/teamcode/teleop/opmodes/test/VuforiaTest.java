package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp(group="Test")
//@Disabled
public class VuforiaTest extends SnapshotTest {

    private Vuforia vuforia;

    @Override
    protected void initialize() {
        super.initialize();
        vuforia = Vuforia.createInstance(VisionSystem.CameraType.FRONT_WEBCAM);
        vuforia.startLook(VisionSystem.TargetType.ALL);
    }

    @Override
    protected void update() {
        super.update();
        Pose pose = vuforia.getPose();
        telemetry.addData("Camera", vuforia.getCameraType().name());
        telemetry.addData("X (in)", GeneralMath.round(pose.x, 3));
        telemetry.addData("Y (in)", GeneralMath.round(pose.y, 3));
        telemetry.addData("R (in)", GeneralMath.round(pose.r, 3));
        telemetry.update();
    }

    @Override
    void onClick() {
        AutoRunner.log("TestVuforiaPose", vuforia.getPose());
    }

    @Override
    protected void onStop() {
        super.onStop();
        vuforia.stopLook();
    }
}
