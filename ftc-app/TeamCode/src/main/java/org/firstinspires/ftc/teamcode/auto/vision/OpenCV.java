package org.firstinspires.ftc.teamcode.auto.vision;

import android.content.Context;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

public class OpenCV implements VisionSystem {

    public static final Rect CAMERA_RECT = new Rect(0, 0, 320, 240);
    public enum CameraType { PHONE, WEBCAM }

    private CameraType cameraType = CameraType.WEBCAM;

    private OpenCvCamera openCvCamera;
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
        openCvCamera.closeCameraDevice();
    }

    @Override
    public boolean found() {
        return detector.isDetected();
    }

    private void initializeCamera() {
        initializeCamera(cameraType);
    }

    private void initializeCamera(CameraType cameraType) {
        this.cameraType = cameraType;
        HardwareMap hardwareMap = Robot.getInstance().hardwareMap;

        switch (cameraType) {
            case PHONE:
                Context context = hardwareMap.appContext;
                int cameraMonitorViewId = context.getResources().getIdentifier("cameraMonitorViewId", "id", context.getPackageName());
                openCvCamera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
                break;
            case WEBCAM:
                openCvCamera = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
                break;
            default:
                throw new IllegalArgumentException("TeamCode OpenCV: Need camera type");
        }
        openCvCamera.openCameraDevice();
    }

    private void startCamera() {
        openCvCamera.setPipeline(detector);
        openCvCamera.startStreaming(CAMERA_RECT.width, CAMERA_RECT.height, OpenCvCameraRotation.UPRIGHT);
    }

}
