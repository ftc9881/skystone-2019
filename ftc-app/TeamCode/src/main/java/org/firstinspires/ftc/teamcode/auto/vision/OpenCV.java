package org.firstinspires.ftc.teamcode.auto.vision;

import android.content.Context;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.CameraManagerInternal;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

public class OpenCV implements VisionSystem {

    public static final Rect CAMERA_RECT = new Rect(0, 0, 320, 240);

    private OpenCvCamera openCvCamera;
    private Camera camera;
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
        HardwareMap hardwareMap = Robot.getInstance().hardwareMap;
        Context context = hardwareMap.appContext;
        int cameraMonitorViewId = context.getResources().getIdentifier("cameraMonitorViewId", "id", context.getPackageName());

        // Phone Camera
//        openCvCamera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//        openCvCamera.openCameraDevice();

        // USB Camera
        openCvCamera = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        openCvCamera.openCameraDevice();

        // asdklfjkl
//        CameraName cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
//        CameraManagerInternal cameraManager = (CameraManagerInternal) ClassFactory.getInstance().getCameraManager();
//        camera = cameraManager.requestPermissionAndOpenCamera(new Deadline(60, TimeUnit.SECONDS), cameraName, null);
//        camera = cameraManager.asyncOpenCameraAssumingPermission(cameraName);
    }

    private void startCamera() {
        openCvCamera.setPipeline(detector);
        openCvCamera.startStreaming(CAMERA_RECT.width, CAMERA_RECT.height, OpenCvCameraRotation.UPRIGHT);
    }

}
