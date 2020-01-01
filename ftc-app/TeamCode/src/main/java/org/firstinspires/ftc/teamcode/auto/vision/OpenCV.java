package org.firstinspires.ftc.teamcode.auto.vision;

import android.content.Context;

import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

public class OpenCV implements VisionSystem {

    public static final Rect CAMERA_RECT = new Rect(0, 0, 320, 240);

    private OpenCvCamera camera;
    public CustomSkystoneDetector detector;

    @Override
    public void initialize() {
        initializeCamera();
    }

    @Override
    public void startLook(TargetType targetType) {
        switch (targetType) {
            case NONE_JUST_RUN_FOREVER:
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
        HardwareMap hardwareMap = Robot.getInstance().hardwareMap;
        Context context = hardwareMap.appContext;
        int cameraMonitorViewId = context.getResources().getIdentifier("cameraMonitorViewId", "id", context.getPackageName());

        // Phone Camera
//        camera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // USB Camera
        camera = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.openCameraDevice();
    }

    private void startCamera() {
        camera.setPipeline(detector);
        camera.startStreaming(CAMERA_RECT.width, CAMERA_RECT.height, OpenCvCameraRotation.UPRIGHT);
    }

}
