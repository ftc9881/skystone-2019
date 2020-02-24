package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.vision.OpenCVThroughVuforia;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;
import org.opencv.core.Rect;

import static org.firstinspires.ftc.teamcode.auto.vision.VisionSystem.CameraType.FRONT_WEBCAM;

@TeleOp(group="Test")
//@Disabled
public class OpenCVThroughVuforiaTest extends SnapshotTest {

    Vuforia vuforia;
    OpenCVThroughVuforia openCV;

    @Override
    protected void initialize() {
        super.initialize();

        openCV = OpenCVThroughVuforia.createInstance(config, FRONT_WEBCAM);
        openCV.startLook(VisionSystem.TargetType.SKYSTONE);

        vuforia = Vuforia.getInstance();
        vuforia.startLook(VisionSystem.TargetType.ALL);
    }

    @Override
    protected void update() {
        super.update();

        Rect rect = openCV.getFoundRect();
        telemetry.addData("Rect", rect.toString());
        telemetry.addData("CenterX", rect.x + rect.width / 2);

        Pose pose = vuforia.getPose();
        telemetry.addData("Camera", vuforia.getCameraType().name());
        telemetry.addData("X (in)", GeneralMath.round(pose.x, 3));
        telemetry.addData("Y (in)", GeneralMath.round(pose.y, 3));
        telemetry.addData("R (in)", GeneralMath.round(pose.r, 3));

        telemetry.update();
    }

    @Override
    void onClick() {
        openCV.writeCurrentImage();
    }

}
