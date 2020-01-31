package org.firstinspires.ftc.teamcode.robot.devices;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.GeneralMath;

import java.util.ArrayList;
import java.util.List;

public class OptimizedIMU {

    private List<BNO055IMU> delegates;
    private AngleUnit angleUnit;

    public OptimizedIMU(HardwareMap hardwareMap, AngleUnit angleUnit) {
        this.angleUnit = angleUnit;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = angleUnit == AngleUnit.DEGREES ? BNO055IMU.AngleUnit.DEGREES : BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

//        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        // TODO: Put delta heading method inside here on a thread
//        List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);
        List<LynxModule> modules = new ArrayList<>();
        LynxModule testModule = hardwareMap.getAll(LynxModule.class).get(0);
        AutoRunner.log("LynxModule?", testModule.getConnectionInfo());
        modules.add(testModule);

        delegates = new ArrayList<>();
        for (LynxModule module : modules) {
            BNO055IMU imu = new LynxEmbeddedIMU(OptimizedI2cDevice.createLynxI2cDeviceSynch(module, 0));
            imu.initialize(parameters);
            delegates.add(imu);
        }
    }

    public Angle getHeading() {
        List<Number> angles = new ArrayList<>();
        for (BNO055IMU delegate : delegates) {
            double reading = delegate.getAngularOrientation().firstAngle;
            // Reverse because we want positive to be right
            angles.add(-reading);
        }
        return new Angle(GeneralMath.mean(angles), angleUnit);
    }

}
