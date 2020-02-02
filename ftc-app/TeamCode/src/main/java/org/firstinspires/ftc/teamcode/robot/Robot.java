package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.hardware.imu.OptimizedIMU;

public class Robot {

    // Singleton pattern; constructor is private and enable only one instance at a time
    private static Robot instance;

    public static Robot newInstance(LinearOpMode opMode) {
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
        opMode.msStuckDetectInit = 3000;
        opMode.msStuckDetectInitLoop = 3000;

        hardwareMap = opMode.hardwareMap;
        imu = new OptimizedIMU(hardwareMap, opMode);

        driveTrain = new DriveTrain(hardwareMap);
    }

}
