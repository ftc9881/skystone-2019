package org.firstinspires.ftc.teamcode.auto.vision;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Command;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public class OpenCV implements VisionSystem {

    public static final Rect CAMERA_RECT = new Rect(0, 0, 320, 240);
    public enum CameraType { PHONE, WEBCAM }

    private CameraType cameraType = CameraType.WEBCAM;

    private OpenCvCamera openCvCamera;
    public SkystoneDetector detector;

    @Override
    public void initialize() {
        initializeCamera();
    }

    @Override
    public void startLook(TargetType targetType) {

        switch (targetType) {
            case NONE_JUST_RUN_FOREVER:
            case SKYSTONE: {
                detector = new SkystoneDetector();
                detector.useDefaults();
                detector.flipImage = cameraType == CameraType.WEBCAM;
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

    public SkystonePosition identifyPosition(LinearOpMode opMode, Command config) {
        List<Integer> foundPositions = new ArrayList<>();
        double stdDev = 999;
        int centerX = 0;

        int stdDevThreshold = config.getInt("std dev threshold", 10);
        int maxDetectList = config.getInt("max detection list size", 10);

        while (opMode.opModeIsActive() && (centerX == 0 || stdDev > stdDevThreshold)) {
            Rect foundRect = detector.foundRectangle();
            centerX = foundRect.x + foundRect.width / 2;
            foundPositions.add(centerX, 0);
            if (foundPositions.size() >= maxDetectList) {
                foundPositions.remove(foundPositions.size()-1);
            }
            stdDev = GeneralMath.standardDeviation(foundPositions);
            AutoRunner.log("SkystonePixel", centerX);
        }

        int averageCenterX = (int) GeneralMath.mean(foundPositions);
        int skystoneLeftBound = config.getInt("skystone left", 0);
        int skystoneRightBound = config.getInt("skystone right", 0);
        if (averageCenterX < skystoneLeftBound) {
            return VisionSystem.SkystonePosition.LEFT;
        } else if (averageCenterX > skystoneRightBound) {
            return VisionSystem.SkystonePosition.RIGHT;
        }
        return VisionSystem.SkystonePosition.CENTER;
    }

}
