package org.firstinspires.ftc.teamcode.auto.vision;

import android.content.Context;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
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
            case RUN_FOREVER:
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
        detector.blobDistanceThreshold = config.getInt("stone blob distance", 60);
        detector.cropRect.x = config.getInt("crop x", 0);
        detector.cropRect.y = config.getInt("crop y", 0);
        detector.cropRect.width = config.getInt("crop w", 0);
        detector.cropRect.height= config.getInt("crop h", 0);

        int stdDevThreshold = config.getInt("std dev threshold", 10);
        int maxDetectList = config.getInt("max detection list size", 10);
        double stdDev = 999;
        int centerX = 0;
        List<Integer> foundPositions = new ArrayList<>();

        while (!opMode.isStopRequested() && (centerX == 0 || stdDev > stdDevThreshold)) {
            Rect foundRect = detector.foundRectangle();
            centerX = foundRect.x + foundRect.width / 2;
            if (foundPositions.size() >= maxDetectList) {
                foundPositions.remove(0);
            }
            foundPositions.add(centerX);
            stdDev = GeneralMath.standardDeviation(foundPositions);
        }

        // Debug
        writeCurrentImage();

        int averageCenterX = (int) GeneralMath.mean(foundPositions);
        AutoRunner.log("Mean", averageCenterX);
        int skystoneLeftBound = config.getInt("skystone left", 0);
        int skystoneRightBound = config.getInt("skystone right", 0);
        if (averageCenterX < skystoneLeftBound) {
            return VisionSystem.SkystonePosition.LEFT;
        } else if (averageCenterX > skystoneRightBound) {
            return VisionSystem.SkystonePosition.RIGHT;
        }
        return VisionSystem.SkystonePosition.CENTER;
    }

    public void writeCurrentImage() {
        Mat mat = detector.getRenderMat(1);
        Mat newMat = new Mat();
        SimpleDateFormat formatter = new SimpleDateFormat("MM-dd-yyyy_HH:mm:ss.SSS");
        Date date = new Date(System.currentTimeMillis());
        String path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES) + "/" + formatter.format(date) + ".png";
        Imgproc.cvtColor(mat, newMat, Imgproc.COLOR_BGR2RGB);
        boolean success = Imgcodecs.imwrite(path, newMat);
        AutoRunner.log("OpenCV", success);
        AutoRunner.log("OpenCV", "Wrote image " + path);
    }

}
