package org.firstinspires.ftc.teamcode.auto.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.SwitchableCameraName;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.math.Pose;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


public class Vuforia implements VisionSystem {

    public enum Target {
        SKYSTONE,
        BLUE_REAR_BRIDGE, RED_REAR_BRIDGE, RED_FRONT_BRIDGE, BLUE_FRONT_BRIDGE,
        RED_PERIMETER_1, RED_PERIMETER_2, FRONT_PERIMETER_1, FRONT_PERIMETER_2,
        BLUE_PERIMETER_1, BLUE_PERIMETER_2, REAR_PERIMETER_1, REAR_PERIMETER_2;
    }

    // Our Vuforia API key from https://developer.vuforia.com/license-manager
    private static final String VUFORIA_KEY = "Ae3SQJr/////AAABma3GOKdfqERdhlPySYr2gGUY3kvAp0ejLY/ecvFSOpdWOYZoJOPtvYn+rL6FuVLK8lfHLQUV+cLOkA91juLovXsGF9SR4EiANCZu3nrp+6iQH/fCcCouQ8m8y+ahDOkW5gkEiu68ioVozoUJ7QBe075/9bnywlrzswoLdQHYEStUeU4OGyTPl2ABapRzdckUpkORgBi/jmBMKvk1IeATeXGqXWY3Xa2ZbB/abA2+1xgS2HcAsC0Xs6wW4O4RWr501Q5GF9GDlAblt0YPQbOXKhLGu3MIB7oEDNYqebu7pYQPy5LiFjc8PgjLcW06OJ0G9jDCaRJToLWGCsqD4dhTn7Nt4eKExBBNMnUSlWTHq0Jm";

    private static final float mmPerInch = 25.4f;

    private VuforiaLocalizer vuforiaLocalizer = null;
    private VuforiaLocalizer.Parameters parameters = null;
    private VuforiaTrackables trackables = null;

    private CameraType cameraType;
    private SwitchableCamera switchableCamera;
    private boolean initialized = false;
    private boolean found = false;
    private Pose lastPose = new Pose();
    private LookAction lookAction;

    private HardwareMap hardwareMap;

    private static Vuforia instance;

    public static Vuforia createInstance(HardwareMap hardwareMap, CameraType cameraType) {
        instance = new Vuforia(hardwareMap, cameraType);
        return instance;
    }

    public static Vuforia getInstance() {
        return instance;
    }

    private Vuforia(HardwareMap hardwareMap, CameraType cameraType) {
        this.cameraType = cameraType;
        this.hardwareMap = hardwareMap;
        initialize();
    }

    private void initialize() {
        startEngine();
        loadTrackables();
        orientTrackablesAtZero();
        applyCameraOrientation();
        initialized = true;
        AutoRunner.log("Vuforia", "Initialization complete");
    }

    @Override
    public void startLook(TargetType target) {
        found = false;
        lastPose = new Pose();
        if (!initialized) {
            AutoRunner.log("Vuforia", "Not initialized; initializing now");
            initialize();
        }
        lookAction = new LookAction(target);
        lookAction.start();
    }

    @Override
    public void stopLook() {
        lookAction.stop();
    }

    @Override
    public boolean found() {
        return found;
    }

    public Pose getPose() {
        return lastPose;
    }

    public void setActiveCamera(CameraType cameraType) {
        found = false;
        lastPose = new Pose();
        if (switchableCamera != null) {
            this.cameraType = cameraType;
            switchableCamera.setActiveCamera(hardwareMap.get(WebcamName.class, cameraType.name));
            applyCameraOrientation();
        }
    }

    public CameraType getCameraType() {
        return cameraType;
    }

    private void startEngine() {
        parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        if (isWebcam()) {
            CameraName activeWebcam = hardwareMap.get(WebcamName.class, cameraType.name);
            CameraName inactiveWebcam = hardwareMap.get(WebcamName.class, cameraType == CameraType.FRONT_WEBCAM ? CameraType.BACK_WEBCAM.name : CameraType.FRONT_WEBCAM.name);
            SwitchableCameraName switchableCameraName = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(activeWebcam, inactiveWebcam);
//            parameters.cameraName = hardwareMap.get(WebcamName.class, cameraType.name);
            parameters.cameraName = switchableCameraName;
            vuforiaLocalizer = ClassFactory.getInstance().createVuforia(parameters);
            switchableCamera = (SwitchableCamera) vuforiaLocalizer.getCamera();
            switchableCamera.setActiveCamera(activeWebcam);
        } else {
            parameters.cameraDirection = BACK;
            parameters.cameraMonitorViewIdParent = hardwareMap.appContext.getResources().getIdentifier(
                    "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            vuforiaLocalizer = ClassFactory.getInstance().createVuforia(parameters);
        }
    }

    private boolean isWebcam() {
        return cameraType != CameraType.PHONE;
    }

    private void loadTrackables() {
        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        trackables = vuforiaLocalizer.loadTrackablesFromAsset("Skystone");
    }

    private void orientTrackablesAtZero() {
        for (VuforiaTrackable trackable : trackables) {
            trackable.setLocation(OpenGLMatrix
                    .translation(0, 0, 0)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 ,-90)));
        }
    }

    private void applyCameraOrientation() {
        List<VuforiaTrackable> allTrackables = new ArrayList<>(trackables);
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(getPositionMatrix(CameraType.PHONE), parameters.cameraDirection);
            if (isWebcam()) {
                CameraName frontWebcam = hardwareMap.get(WebcamName.class, CameraType.FRONT_WEBCAM.name);
                CameraName backWebcam = hardwareMap.get(WebcamName.class, CameraType.BACK_WEBCAM.name);
                ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(frontWebcam, getPositionMatrix(CameraType.FRONT_WEBCAM));
                ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(backWebcam, getPositionMatrix(CameraType.BACK_WEBCAM));
            }
        }
    }
    
    private OpenGLMatrix getPositionMatrix(CameraType cameraType) {
        return OpenGLMatrix
            .translation((float)cameraType.displacement.y, (float)cameraType.displacement.x, (float)cameraType.displacement.r)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, (float)cameraType.rotation.y, (float)cameraType.rotation.r, (float)cameraType.rotation.x));
    }


    class LookAction extends Action {
        private boolean targetVisible;
        private TargetType targetType;
        private OpenGLMatrix lastLocation;
        private ArrayList<VuforiaTrackable> targetTrackables;

        LookAction(TargetType targetType) {
            this.targetType = targetType;
        }

        @Override
        protected void onRun() {
            trackables.activate();
            targetVisible = false;
            lastLocation = null;
            targetTrackables = getTargetTrackables();
        }

        @Override
        protected void insideRun() {
            for (VuforiaTrackable trackable : targetTrackables) {
                targetVisible = ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible();
                if (targetVisible) {
                    OpenGLMatrix robotTransform = getRobotTransform(trackable);
                    lastLocation = robotTransform != null ? robotTransform : lastLocation;
                    setLastPose();
                    break;
                }
            }
        }

        @Override
        protected boolean runIsComplete() {
            return false;
        }

        @Override
        protected void onEndRun() {
            trackables.deactivate();
        }

        private void setLastPose() {
            if (lastLocation != null) {
                double x = lastLocation.getTranslation().get(1) / mmPerInch;
                double y = lastLocation.getTranslation().get(0) / mmPerInch;
                double r = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES).thirdAngle;
                lastPose = new Pose(x, y, r);
            }
        }

        private ArrayList<VuforiaTrackable> getTargetTrackables() {
            ArrayList<VuforiaTrackable> trackingTrackables = new ArrayList<VuforiaTrackable>();
            switch (targetType) {
                case SKYSTONE:
                    trackingTrackables.add(trackables.get(Target.SKYSTONE.ordinal()));
                    break;
                case BRIDGE:
                    trackingTrackables.add(trackables.get(Target.BLUE_REAR_BRIDGE.ordinal()));
                    trackingTrackables.add(trackables.get(Target.RED_REAR_BRIDGE.ordinal()));
                    trackingTrackables.add(trackables.get(Target.RED_FRONT_BRIDGE.ordinal()));
                    trackingTrackables.add(trackables.get(Target.BLUE_FRONT_BRIDGE.ordinal()));
                    break;
                case PERIMETER:
                    trackingTrackables.add(trackables.get(Target.RED_PERIMETER_1.ordinal()));
                    trackingTrackables.add(trackables.get(Target.RED_PERIMETER_2.ordinal()));
                    trackingTrackables.add(trackables.get(Target.FRONT_PERIMETER_1.ordinal()));
                    trackingTrackables.add(trackables.get(Target.FRONT_PERIMETER_2.ordinal()));
                    trackingTrackables.add(trackables.get(Target.BLUE_PERIMETER_1.ordinal()));
                    trackingTrackables.add(trackables.get(Target.BLUE_PERIMETER_2.ordinal()));
                    trackingTrackables.add(trackables.get(Target.REAR_PERIMETER_1.ordinal()));
                    trackingTrackables.add(trackables.get(Target.REAR_PERIMETER_2.ordinal()));
                    break;
                default:
                    trackingTrackables.addAll(trackables);
                    break;
            }
            return trackingTrackables;
        }

        private OpenGLMatrix getRobotTransform(VuforiaTrackable trackable) {
            VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) trackable.getListener();
            return listener.getUpdatedRobotLocation();
        }

    }

}
