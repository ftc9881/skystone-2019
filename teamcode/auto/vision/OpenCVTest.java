package org.firstinspires.ftc.teamcode.auto.vision;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.TeleOpBase;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp
public class OpenCVTest extends TeleOpBase {
    private SkystoneDetector detector;
    OpenCvCamera phoneCam;

    @Override
    protected void initialize() {
        detector = new SkystoneDetector();
        detector.useDefaults();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();

        phoneCam.setPipeline(detector);

        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    @Override
    protected void update() {
        telemetry.addData("Rect Area", detector.foundRectangle().area());
        telemetry.addData("Frame Count", phoneCam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
        telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
        telemetry.update();
    }

}
