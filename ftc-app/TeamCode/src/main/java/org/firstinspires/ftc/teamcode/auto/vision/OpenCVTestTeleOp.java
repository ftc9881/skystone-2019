package org.firstinspires.ftc.teamcode.auto.vision;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.TeleOpBase;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import static org.firstinspires.ftc.teamcode.auto.vision.OpenCV.CAMERA_RECT;

@TeleOp(name="OpenCV")
public class OpenCVTestTeleOp extends TeleOpBase {

    private CustomSkystoneDetector skystoneDetector;
    private OpenCvCamera phoneCam;


    @Override
    protected void initialize() {

        Rect lookWindow = new Rect();
        lookWindow.x = config.getInt("look window x", CAMERA_RECT.x);
        lookWindow.y = config.getInt("look window y", CAMERA_RECT.y);
        lookWindow.width = config.getInt("look window w", CAMERA_RECT.width);
        lookWindow.height = config.getInt("look window h", CAMERA_RECT.height);
        int maxBlobDistance = config.getInt("blob distance", 50);

        skystoneDetector = new CustomSkystoneDetector();
        skystoneDetector.lookWindow = lookWindow;
        skystoneDetector.maxBlobDistance = maxBlobDistance;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(skystoneDetector);
        phoneCam.startStreaming(CAMERA_RECT.width, CAMERA_RECT.height, OpenCvCameraRotation.UPRIGHT);

    }

    @Override
    protected void update() {
        telemetry.addData("Rect Area", skystoneDetector.foundRectangle().area());
        telemetry.addData("Rect X", skystoneDetector.foundRectangle().x);
        telemetry.addData("Rect Y", skystoneDetector.foundRectangle().y);
        telemetry.addData("Rect W", skystoneDetector.foundRectangle().width);
        telemetry.addData("Rect H", skystoneDetector.foundRectangle().height);
        telemetry.update();
    }

}
