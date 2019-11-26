package org.firstinspires.ftc.teamcode.auto.vision;

import android.content.Context;

import com.disnodeteam.dogecv.detectors.DogeCVDetector;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class OpenCV implements VisionSystem {

    public static final Rect CAMERA_RECT = new Rect(0, 0, 320, 240);

    private OpenCvCamera camera;
    DogeCVDetector detector;

    @Override
    public void initialize() {
        initializeCamera();
    }

    @Override
    public void startLook(TargetType targetType) {
        switch (targetType) {
            case SKYSTONE: {
                detector = new CustomSkystoneDetector();
                detector.useDefaults();
            }
            default: {

            }
        }
        startCamera();
    }

    @Override
    public void stopLook() {
        camera.closeCameraDevice();
    }

    @Override
    public boolean found() {
        return detector.isDetected();
    }

    private void initializeCamera() {
        Context context = Robot.getInstance().hardwareMap.appContext;
        int cameraMonitorViewId = context.getResources().getIdentifier("cameraMonitorViewId", "id", context.getPackageName());
        camera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        camera.openCameraDevice();
    }

    private void startCamera() {
        camera.setPipeline(detector);
        camera.startStreaming(CAMERA_RECT.width, CAMERA_RECT.height, OpenCvCameraRotation.UPRIGHT);
    }

}
