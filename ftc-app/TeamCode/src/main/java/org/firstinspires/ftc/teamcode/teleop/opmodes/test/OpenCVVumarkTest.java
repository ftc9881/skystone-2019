package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.vision.OpenCV;
import org.firstinspires.ftc.teamcode.auto.vision.OpenCVDetector;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;
import org.opencv.core.Rect;

@TeleOp(group="Test")
@Disabled
public class OpenCVVumarkTest extends SnapshotTest {

    OpenCV openCV;

    @Override
    protected void initialize() {
        super.initialize();

        openCV = new OpenCV(config, VisionSystem.CameraType.FRONT_WEBCAM);
        openCV.setConfig(config);
        openCV.cameraType = VisionSystem.CameraType.FRONT_WEBCAM;
        openCV.startLook(VisionSystem.TargetType.PERIMETER);
    }

    @Override
    protected void update() {
        super.update();

        Rect rect = openCV.getFoundRect();
        telemetry.addData("Vumark", rect.area() > 0);
        telemetry.addData("Width", rect.width);
        telemetry.addData("Rect", rect);
        telemetry.addData("Pose", openCV.getPose());
        telemetry.update();
    }

    @Override
    void onClick() {
        openCV.writeCurrentImage(OpenCVDetector.Stage.THRESHOLD);
        openCV.writeCurrentImage(OpenCVDetector.Stage.DISPLAY);
        openCV.writeCurrentImage(OpenCVDetector.Stage.DEBUG);
        openCV.writeCurrentImage(OpenCVDetector.Stage.RAW_IMAGE);
    }

    @Override
    protected void onStop() {
        super.onStop();
        openCV.stopLook();
    }
}
