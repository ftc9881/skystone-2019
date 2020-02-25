package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.hardware.imu.OptimizedIMU;

public class Robot {

    private static Robot instance;

    public static Robot newInstance(LinearOpMode opMode) {
        /*
            CD: It's problematic to create an instance with the opMode as an argument since the opmode can
                change based on the user selection and this is a static instance.  Since you don't actually
                use a singleton (see second comment) it isn't a problem, but when you fix that it will be.
        */
        /*
            CD: In singleton you should check to see if the static instance is null before doing the lazy instantiation here
         */
        instance = new Robot(opMode);
        return instance;
    }

    public static Robot getInstance() {
        return instance;
    }

    public LinearOpMode opMode;
    public DriveTrain driveTrain;
    public OptimizedIMU imu;
    public Pose currentPose;

    protected Robot() {}

    protected Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        driveTrain = new DriveTrain(getHardwareMap());
        initializeOpMode();
    }

    private void initializeOpMode() {
        opMode.msStuckDetectInit = 3000;
        opMode.msStuckDetectInitLoop = 3000;
    }

    public HardwareMap getHardwareMap() {
        return opMode.hardwareMap;
    }

    public void initializeIMU() {
        imu = new OptimizedIMU(opMode.hardwareMap, opMode);
    }

    public boolean imuIsInitialized() {
        return imu != null;
    }

}
