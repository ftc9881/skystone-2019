package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotSystemFactory {

    HardwareMap hardwareMap;

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

    public FoundationGrabber foundationGrabber() {
        return new FoundationGrabber(hardwareMap);
    }

    public SensorSystem sensorSystem() {
        return new SensorSystem(hardwareMap);
    }

    public OdometrySystem odometrySystem() {
        return new OdometrySystem(hardwareMap);
    }

}
