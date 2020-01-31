package org.firstinspires.ftc.teamcode.auto.vision;

import android.content.Context;
import android.graphics.Path;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;
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
    public SkystoneDetector detector;

    public CameraType cameraType = CameraType.FRONT_WEBCAM;
    protected Configuration config;
    protected OpenCvCamera openCvCamera;

    public static OpenCV getInstance() {
        return new OpenCV();
    }

    private OpenCV() {
        initializeCamera(cameraType);
        config = new Configuration("Vision");
    }

    @Override
    public void startLook(TargetType targetType) {

        switch (targetType) {
            case ALL:
            case SKYSTONE: {
                detector = new SkystoneDetector();
                detector.useDefaults();
                detector.flipImage = cameraType == CameraType.FRONT_WEBCAM;
            }
            default: {

            }
        }
        if (detector != null) {
            setDetectorConfig(config);
        }
        startCamera();
    }

    @Override
    public void stopLook() {
        openCvCamera.stopStreaming();
        openCvCamera.closeCameraDevice();
    }

    public void setDetectorConfig(Command config) {
        detector.yellowBlobbingThreshold = config.getInt("yellow blobbing", detector.yellowBlobbingThreshold);
        detector.blackBlobbingThreshold = config.getInt("black blobbing", detector.blackBlobbingThreshold);
        detector.minimumArea = config.getInt("min area", detector.minimumArea);
        detector.cropRect.x = config.getInt("crop x", detector.cropRect.x);
        detector.cropRect.y = config.getInt("crop y", detector.cropRect.y);
        detector.cropRect.width = config.getInt("crop w", detector.cropRect.width);
        detector.cropRect.height= config.getInt("crop h", detector.cropRect.height);
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
            case FRONT_WEBCAM:
            case BACK_WEBCAM:
                openCvCamera = new OpenCvWebcam(hardwareMap.get(WebcamName.class, cameraType.name));
                break;
            default:
                throw new IllegalArgumentException("TeamCode@OpenCV: Need camera type");
        }
        openCvCamera.openCameraDevice();
    }

    protected void startCamera() {
        openCvCamera.setPipeline(detector);
        openCvCamera.startStreaming(CAMERA_RECT.width, CAMERA_RECT.height, OpenCvCameraRotation.UPRIGHT);
    }


    @Override
    public SkystonePosition identifySkystonePosition() {
        Configuration config = new Configuration("Vision");
        LinearOpMode opMode = Robot.getInstance().opMode;

        int skystoneLeftBound = config.getInt("skystone left", 0);
        int skystoneRightBound = config.getInt("skystone right", 0);
        int listSize = config.getInt("list size", 10);
        int centerX = 0;
        List<Number> foundPositions = new ArrayList<>();

        while (!opMode.isStopRequested() && !opMode.isStarted() && centerX == 0) {
            Rect foundRect = detector.foundRectangle();
            centerX = foundRect.x + foundRect.width / 2;
            if (foundPositions.size() >= listSize) {
                foundPositions.remove(0);
            }
            foundPositions.add(centerX);
        }

        // Debug
        writeCurrentImage();

        int averageCenterX = (int) GeneralMath.mean(foundPositions);
        AutoRunner.log("Mean", averageCenterX);
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
