package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.hardware.imu.OptimizedIMU;

public class Robot {

    // Singleton pattern; constructor is private and enable only one instance at a time
    private static Robot instance;

    public static Robot newInstance(LinearOpMode opMode) {
        /*
            CD: It's problemastic to create an instance with the opMode as an argument since  the opmode can
                change based on the user selection and this is a static instance.  Since you don't actually
                use a singleton (see second comment) it isen't a problem, but when you fix that it will be.
        */

        /*
            CD: In singleton you should check to see if the static instance is null before doign the lazy instantiation here
         */
        instance = new Robot(opMode);
        return instance;
    }

    public static Robot getInstance() {
        return instance;
    }

    public LinearOpMode opMode;
    public HardwareMap hardwareMap;
    public DriveTrain driveTrain;
    public OptimizedIMU imu;
    public Pose currentPose;


    protected Robot() {}

    protected Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        /*
            CD: This should be done in an intitialization method
         */
        opMode.msStuckDetectInit = 3000;
        opMode.msStuckDetectInitLoop = 3000;

        /*
            CD: Since the hardware map is a field of opMode, you should just have opMode as a field and then wrap it with Robot.getHardwareMap() { return opMode.getHardwareMap(); }
         */
        hardwareMap = opMode.hardwareMap;

        driveTrain = new DriveTrain(hardwareMap);
    }

    public void initializeIMU() {
        imu = new OptimizedIMU(hardwareMap, opMode);
    }

}
