package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.vision.OpenCV;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;

public class RobotSystemFactory {

    public VisionSystem.Type visionSystemType = VisionSystem.Type.OPENCV;
    private HardwareMap hardwareMap;

    public RobotSystemFactory(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public DriveTrain driveTrain() {
        return new DriveTrain(hardwareMap);
    }

    public Arm arm() {
        return new Arm(hardwareMap);
    }

    public Intake intakeSystem() {
        return new Intake(hardwareMap);
    }

    public SensorSystem sensorSystem() {
        return new SensorSystem(hardwareMap);
    }

    public OdometrySystem odometrySystem() {
        return new OdometrySystem(hardwareMap);
    }

    public VisionSystem visionSystem() {
        switch (visionSystemType) {
            case OPENCV:
                return new OpenCV();
            case VUFORIA:
                return new Vuforia(hardwareMap.appContext);
        }
        return null;
    }

}
