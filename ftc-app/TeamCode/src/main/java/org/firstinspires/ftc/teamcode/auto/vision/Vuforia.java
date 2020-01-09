/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.auto.vision;

import android.content.Context;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


public class Vuforia implements VisionSystem {

    public enum Target {
        SKYSTONE(0),
        BLUE_REAR_BRIDGE(1),
        RED_REAR_BRIDGE(2),
        RED_FRONT_BRIDGE(3),
        BLUE_FRONT_BRIDGE(4),
        RED_PERIMETER_1(5),
        RED_PERIMETER_2(6),
        FRONT_PERIMETER_1(7),
        FRONT_PERIMETER_2(8),
        BLUE_PERIMETER_1(9),
        BLUE_PERIMETER_2(10),
        REAR_PERIMETER_1(11),
        REAR_PERIMETER_2(12);

        public final int index;
        Target(int index) {
            this.index = index;
        }
    }

    // Our Vuforia API key from https://developer.vuforia.com/license-manager
    private static final String VUFORIA_KEY = "Ae3SQJr/////AAABma3GOKdfqERdhlPySYr2gGUY3kvAp0ejLY/ecvFSOpdWOYZoJOPtvYn+rL6FuVLK8lfHLQUV+cLOkA91juLovXsGF9SR4EiANCZu3nrp+6iQH/fCcCouQ8m8y+ahDOkW5gkEiu68ioVozoUJ7QBe075/9bnywlrzswoLdQHYEStUeU4OGyTPl2ABapRzdckUpkORgBi/jmBMKvk1IeATeXGqXWY3Xa2ZbB/abA2+1xgS2HcAsC0Xs6wW4O4RWr501Q5GF9GDlAblt0YPQbOXKhLGu3MIB7oEDNYqebu7pYQPy5LiFjc8PgjLcW06OJ0G9jDCaRJToLWGCsqD4dhTn7Nt4eKExBBNMnUSlWTHq0Jm";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch; // the height of the center of the target image above the floor

    private static final float stoneZ = 2.00f * mmPerInch;
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;
    private static final float bridgeRotZ = 180;
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // TODO: Phone orientation and displacement values
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = true;
    private final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
    private final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
    private final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

    private VuforiaLocalizer vuforiaLocalizer = null;
    private VuforiaLocalizer.Parameters parameters = null;
    private VuforiaTrackables trackables = null;

    private boolean initialized = false;
    private Pose lastPose = null;
    private LookAction lookAction;

    private HardwareMap hardwareMap;
    private Context fileContext;


    public Vuforia(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.fileContext = hardwareMap.appContext;
    }

    @Override
    public void initialize() {
        startEngine(true);
        loadTrackables();
        orientTrackables();
        applyPhoneOrientation();
        trackables.activate();
        initialized = true;
        AutoRunner.log("Vuforia", "Initialization complete");
    }

    @Override
    public void startLook(TargetType target) {
        lastPose = null;
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
        return lastPose != null;
    }

    public Pose getPose() {
        return lastPose;
    }

    private void startEngine(boolean useUsbCamera) {
        parameters = new VuforiaLocalizer.Parameters(fileContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", fileContext.getPackageName()));
        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        if (useUsbCamera) {
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        }
        else {
            parameters.cameraDirection = CAMERA_CHOICE;
        }

        //  Instantiate the Vuforia engine
        vuforiaLocalizer = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void loadTrackables() {
        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        trackables = vuforiaLocalizer.loadTrackablesFromAsset("Skystone");
    }

    private void orientTrackables() {
         /**
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        VuforiaTrackable stoneTarget = trackables.get(Target.SKYSTONE.index);
        stoneTarget.setName("Stone Target");
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));


        //Set the position of the bridge support targets with relation to origin (center of field)
        VuforiaTrackable blueRearBridge = trackables.get(Target.BLUE_REAR_BRIDGE.index);
        blueRearBridge.setName("Blue Rear Bridge");
        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        VuforiaTrackable redRearBridge = trackables.get(Target.RED_REAR_BRIDGE.index);
        redRearBridge.setName("Red Rear Bridge");
        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        VuforiaTrackable redFrontBridge = trackables.get(Target.RED_FRONT_BRIDGE.index);
        redFrontBridge.setName("Red Front Bridge");
        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        VuforiaTrackable blueFrontBridge = trackables.get(Target.BLUE_FRONT_BRIDGE.index);
        blueFrontBridge.setName("Blue Front Bridge");
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));


        //Set the position of the perimeter targets with relation to origin (center of field)
        VuforiaTrackable red1 = trackables.get(Target.RED_PERIMETER_1.index);
        red1.setName("Red Perimeter 1");
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        VuforiaTrackable red2 = trackables.get(Target.RED_PERIMETER_2.index);
        red2.setName("Red Perimeter 2");
        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        VuforiaTrackable front1 = trackables.get(Target.FRONT_PERIMETER_1.index);
        front1.setName("Front Perimeter 1");
        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        VuforiaTrackable front2 = trackables.get(Target.FRONT_PERIMETER_2.index);
        front2.setName("Front Perimeter 2");
        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        VuforiaTrackable blue1 = trackables.get(Target.BLUE_PERIMETER_1.index);
        blue1.setName("Blue Perimeter 1");
        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        VuforiaTrackable blue2 = trackables.get(Target.BLUE_PERIMETER_2.index);
        blue2.setName("Blue Perimeter 2");
        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        VuforiaTrackable rear1 = trackables.get(Target.REAR_PERIMETER_1.index);
        rear1.setName("Rear Perimeter 1");
        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        VuforiaTrackable rear2 = trackables.get(Target.REAR_PERIMETER_2.index);
        rear2.setName("Rear Perimeter 2");
        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
    }

    private void applyPhoneOrientation() {
        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(trackables);

        // Create a transformation matrix describing where the phone is on the robot.
        // Make sure phone orientation is locked.
        float phoneYRotate = CAMERA_CHOICE == BACK ? -90 : 90;
        float phoneXRotate = PHONE_IS_PORTRAIT ? 90 : 0;
        float phoneZRotate = 0;

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        // Let all the trackable listeners know where the phone is.
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
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
                    break;
                }
            }
        }

        @Override
        protected boolean runIsComplete() {
            return targetVisible && targetType != TargetType.NONE_JUST_RUN_FOREVER;
        }

        @Override
        protected void onEndRun() {
            if (lastLocation != null) {
                double x = lastLocation.getTranslation().get(0);
                double y = lastLocation.getTranslation().get(1);
                double r = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, RADIANS).thirdAngle;
                lastPose = new Pose(x, y, r);
            }
            trackables.deactivate();
        }


        private ArrayList<VuforiaTrackable> getTargetTrackables() {
            ArrayList<VuforiaTrackable> trackingTrackables = new ArrayList<VuforiaTrackable>();
            switch (targetType) {
                case SKYSTONE:
                    trackingTrackables.add(trackables.get(Target.SKYSTONE.index));
                    break;
                case BRIDGE:
                    trackingTrackables.add(trackables.get(Target.BLUE_REAR_BRIDGE.index));
                    trackingTrackables.add(trackables.get(Target.RED_REAR_BRIDGE.index));
                    trackingTrackables.add(trackables.get(Target.RED_FRONT_BRIDGE.index));
                    trackingTrackables.add(trackables.get(Target.BLUE_FRONT_BRIDGE.index));
                    break;
                case PERIMETER:
                    trackingTrackables.add(trackables.get(Target.RED_PERIMETER_1.index));
                    trackingTrackables.add(trackables.get(Target.RED_PERIMETER_2.index));
                    trackingTrackables.add(trackables.get(Target.FRONT_PERIMETER_1.index));
                    trackingTrackables.add(trackables.get(Target.FRONT_PERIMETER_2.index));
                    trackingTrackables.add(trackables.get(Target.BLUE_PERIMETER_1.index));
                    trackingTrackables.add(trackables.get(Target.BLUE_PERIMETER_2.index));
                    trackingTrackables.add(trackables.get(Target.REAR_PERIMETER_1.index));
                    trackingTrackables.add(trackables.get(Target.REAR_PERIMETER_2.index));
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
