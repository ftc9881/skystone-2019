package org.firstinspires.ftc.teamcode.auto.vision;

import android.content.Context;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.math.Pose;
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
    protected OpenCVDetector detector;

    public CameraType cameraType = CameraType.FRONT_WEBCAM;
    protected Command config;
    protected OpenCvCamera openCvCamera;

    private SkystonePositionIdentifierAction positionDeterminer;

    public void setConfig(Command config) {
        this.config = config;
    }

    public Rect getFoundRect() {
        return detector != null ? detector.getFoundRect() : new Rect();
    }

    public Pose getPose() {
        return detector.getPose();
    }

    public OpenCV(Command config, CameraType cameraType) {
        initializeCamera(cameraType);
        this.config = config;
    }

    @Override
    public void startLook(TargetType targetType) {

        switch (targetType) {
            case SKYSTONE: {
                detector = new SkystoneDetector();
                break;
            }
            case PERIMETER: {
                detector = new VumarkDetectorOld();
                break;
            }
            default: {
                AutoRunner.log("OpenCV", "Unsupported target type :(");
            }
        }
        if (detector != null) {
            detector.setConfig(config);
            startCamera();
        }
    }

    @Override
    public void stopLook() {
        // TODO: Why crash on stop stream?
//        openCvCamera.stopStreaming();
//        openCvCamera.closeCameraDevice();
    }

    private void initializeCamera(CameraType cameraType) {
        this.cameraType = cameraType;
        HardwareMap hardwareMap = Robot.getInstance().getHardwareMap();

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

    public void startIdentifyingSkystonePosition(LinearOpMode opMode) {
        positionDeterminer = new SkystonePositionIdentifierAction(opMode);
        positionDeterminer.start();
    }

    public SkystonePosition getSkystonePosition() {
        return positionDeterminer != null ? positionDeterminer.getPosition() : SkystonePosition.NONE;
    }

    public void writeCurrentImage() {
        writeCurrentImage(OpenCVDetector.Stage.DISPLAY);
    }

    public void writeCurrentImage(OpenCVDetector.Stage stage) {
        Mat mat = detector.getMat(stage);
        writeImage(mat, stage.name());
    }

    public static void writeImage(Mat mat, String tag) {
        if (mat != null && !mat.empty()) {
            Mat newMat = new Mat();
            SimpleDateFormat formatter = new SimpleDateFormat("MM-dd-yyyy_HH:mm:ss.SSS");
            Date date = new Date(System.currentTimeMillis());
            String path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES) + "/" + formatter.format(date) + "_" + tag + ".png";
            Imgproc.cvtColor(mat, newMat, Imgproc.COLOR_BGR2RGB);
            boolean success = Imgcodecs.imwrite(path, newMat);
            if (success) {
                AutoRunner.log("OpenCV", "Wrote image " + path);
            } else {
                AutoRunner.log("OpenCV", "Failed to write image " + path);
            }
        } else {
            AutoRunner.log("OpenCV", "Detector did not have image to write");
        }
    }

    class SkystonePositionIdentifierAction extends Action {
        SkystonePosition position;
        int skystoneLeftBound;
        int skystoneRightBound;
        int listSize;
        int centerX;
        List<Number> foundPositions;

        public SkystonePositionIdentifierAction(LinearOpMode opMode) {
            this.opMode = opMode;
        }

        public SkystonePosition getPosition() {
            return position;
        }

        @Override
        protected void onRun() {
            position = SkystonePosition.NONE;
            skystoneLeftBound = config.getInt("skystone left", 0);
            skystoneRightBound = config.getInt("skystone right", 0);
            listSize = config.getInt("list size", 4);
            foundPositions = new ArrayList<>();
            centerX = 0;
        }

        @Override
        protected void insideRun() {

            if (detector.isGettingImages()) {
                Rect foundRect = detector.getFoundRect();
                centerX = foundRect.x + foundRect.width / 2;
                if (foundPositions.size() >= listSize) {
                    foundPositions.remove(0);
                }
                foundPositions.add(centerX);

                int averageCenterX = (int) GeneralMath.mean(foundPositions);
                if (averageCenterX < skystoneLeftBound) {
                    position = SkystonePosition.LEFT;
                } else if (averageCenterX > skystoneRightBound) {
                    position = SkystonePosition.RIGHT;
                } else {
                    position = SkystonePosition.CENTER;
                }
            } else {
                position = SkystonePosition.NONE;
            }

            AutoRunner.setSkystonePosition(position);
        }

        @Override
        protected boolean runIsComplete() {
            return opMode.isStarted();
        }

        @Override
        protected void onEndRun() {
            writeCurrentImage();
            openCvCamera.closeCameraDevice();
        }

    }

}
