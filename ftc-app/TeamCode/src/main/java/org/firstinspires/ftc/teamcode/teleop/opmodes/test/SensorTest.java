package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.sensors.SharpPair;
import org.firstinspires.ftc.teamcode.sensors.SharpDistanceSensor;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp
public class SensorTest extends TeleOpBase {

    SharpPair blockDetector;
    SharpPair foundationDetector;
    SharpDistanceSensor foundationSensorL;
    SharpDistanceSensor foundationSensorR;
    AnalogInput foundationDetectorInputL;
    AnalogInput foundationDetectorInputR;

    Button autoModeButton = new Button();

    PIDController xPid;
    PIDController yPid;
    PIDController rPid;

    boolean isAutoMode = false;

    @Override
    protected void initialize() {

        /*  currently testing only 1 pair
        // block detection
        AnalogInput baseDetectorInputL = robot.hardwareMap.analogInput.get("base detector L");
        AnalogInput baseDetectorInputR = robot.hardwareMap.analogInput.get("base detector R");
        SharpDistanceSensor baseSensorL = new SharpDistanceSensor(baseDetectorInputL);
        SharpDistanceSensor baseSensorR = new SharpDistanceSensor(baseDetectorInputR);
        */


        // foundation detection
        foundationSensorL = new SharpDistanceSensor(hardwareMap, "foundationDetectorL");
        foundationSensorR = new SharpDistanceSensor(hardwareMap, "foundationDetectorR");

        /*
         blockDetector = new SharpPair(baseSensorL, baseSensorR,
             config.getDouble("blockDetectDist", 21),
             config.getDouble("blockDetectMargin", 2)
         );
        */

         foundationDetector = new SharpPair(foundationSensorL, foundationSensorR,
//             config.getDouble("foundationDetectDist", 10),
//             config.getDouble("foundationDetectMargin", 2)
                 25,
                 1
         );

        double idealXPosition = config.getDouble("sensor X target", 0);
        //x is f/b, y is strafe direction
//        xPid = new PIDController(config, "sensor x", idealXPosition);
//        yPid = new PIDController(config, "sensor y", 25);
//        rPid = new PIDController(config, "sensor r", 0);

        xPid = new PIDController(0.05, 0, 0, idealXPosition);
        yPid = new PIDController(0.05, 0, 0, 25);
        rPid = new PIDController(0.05, 0, 0, 0);
    }

    @Override
    protected void update() {
        autoModeButton.update(gamepad1.a);

        if (autoModeButton.is(Button.State.DOWN)) {
            robot.driveTrain.stop();
            isAutoMode = !isAutoMode;
        }

//        if (inputFromPlayer()) {
//            isAutoMode = false;
//        }

        if (isAutoMode) {
            Pose drivePose = new Pose(0, 0, 0);
            drivePose.r = rPid.getCorrectedOutput(foundationDetector.getDiff());
            if (Math.abs(drivePose.r) < 0.3) {
//                drivePose.x = xPid.getCorrectedOutput(blockDetector.getDistanceAvg());
                drivePose.y = yPid.getCorrectedOutput(foundationDetector.getDistanceAvg());
            }
            AutoRunner.log("SensorTestDriveY", drivePose.y);
            AutoRunner.log("SensorTestDriveR", drivePose.r);
            telemetry.addData("drive y", drivePose.y);
            telemetry.addData("drive r", drivePose.r);
            robot.driveTrain.drive(drivePose);
        }


        telemetry.addData("Auto Align Mode", isAutoMode);
        telemetry.addData("Left  Foundation Dist", foundationDetector.getDistanceL());
        telemetry.addData("Right Foundation Dist", foundationDetector.getDistanceR());
        telemetry.addData("Raw voltage L", foundationDetectorInputL.getVoltage());
        telemetry.addData("Raw voltage R", foundationDetectorInputR.getVoltage());
        telemetry.update();

    }
}
