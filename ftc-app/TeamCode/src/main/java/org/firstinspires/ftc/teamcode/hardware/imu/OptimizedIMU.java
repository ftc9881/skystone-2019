package org.firstinspires.ftc.teamcode.hardware.imu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.GeneralMath;

import java.util.ArrayList;
import java.util.List;

public class OptimizedIMU {

    private List<BNO055IMU> delegates;
    private HeadingIntegrator headingIntegrator;

    public OptimizedIMU(HardwareMap hardwareMap, LinearOpMode opMode) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);
        delegates = new ArrayList<>();
        for (LynxModule module : modules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            BNO055IMU imu = new LynxEmbeddedIMU(OptimizedI2cDevice.createLynxI2cDeviceSynch(module, 0));
            imu.initialize(parameters);
            delegates.add(imu);
        }

        headingIntegrator = new HeadingIntegrator(opMode);
        headingIntegrator.start();
    }

    public Angle getHeading() {
        return new Angle(AngleUnit.normalizeDegrees(getIntegratedHeading().getDegrees()), AngleUnit.DEGREES);
    }

    public Angle getIntegratedHeading() {
        // Reverse because we want positive to be right
        return new Angle(-headingIntegrator.getDegrees(), AngleUnit.DEGREES);
    }

    class HeadingIntegrator extends Action {
        private double cumulativeDegrees;
        private int numberOfImus;
        private List<Number> currentAngles;
        private List<Number> integratedAngles;

        HeadingIntegrator(LinearOpMode opMode) {
            this.opMode = opMode;
        }

        double getDegrees() {
            return cumulativeDegrees;
        }

        @Override
        protected void onRun() {
            AutoRunner.log("Optimized IMU", "Starting integrator");
            cumulativeDegrees = 0;
            numberOfImus = delegates.size();
            currentAngles = new ArrayList<>();
            integratedAngles = new ArrayList<>();
            for (BNO055IMU delegate : delegates) {
                currentAngles.add(delegate.getAngularOrientation().firstAngle);
                integratedAngles.add(0);

            }
        }

        @Override
        protected void insideRun() {
            List<Number> previousAngles = new ArrayList<>(currentAngles);
            for (int i = 0; i < numberOfImus; i++) {
                currentAngles.set(i, delegates.get(i).getAngularOrientation().firstAngle);
                double deltaDegrees = currentAngles.get(i).doubleValue() - previousAngles.get(i).doubleValue();
                if (deltaDegrees > 180) {
                    deltaDegrees -= 360;
                } else if (deltaDegrees < -180) {
                    deltaDegrees += 360;
                }
                integratedAngles.set(i, integratedAngles.get(i).doubleValue() + deltaDegrees);
            }
            cumulativeDegrees = GeneralMath.mean(integratedAngles);
        }

        @Override
        protected boolean runIsComplete() {
            return false;
        }

        @Override
        protected void onEndRun() {
            AutoRunner.log("OptimizedIMU", "Stopped thread");
        }
    }

}
