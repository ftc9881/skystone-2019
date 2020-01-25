package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

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

    boolean isAutoMode;

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
        foundationDetectorInputL = robot.hardwareMap.analogInput.get("foundationDetectorL");
        foundationDetectorInputR = robot.hardwareMap.analogInput.get("foundationDetectorR");
        foundationSensorL = new SharpDistanceSensor(foundationDetectorInputL);
        foundationSensorR = new SharpDistanceSensor(foundationDetectorInputR);

        /*
         blockDetector = new SharpPair(baseSensorL, baseSensorR,
             config.getDouble("blockDetectDist", 21),
             config.getDouble("blockDetectMargin", 2)
         );
        */

         foundationDetector = new SharpPair(foundationSensorL, foundationSensorR,
             config.getDouble("foundationDetectDist", 10),
             config.getDouble("foundationDetectMargin", 2)
         );

         /*
        double idealXPosition = config.getDouble("sensor X target", 0);
        //x is f/b, y is strafe direction
        xPid = new PIDController(config, "sensor x", idealXPosition);
        yPid = new PIDController(config, "sensor y", 0);
        rPid = new PIDController(config, "sensor r", 0);
          */
    }

    @Override
    protected void update() {
        telemetry.addData("Left  Foundation Dist: ", foundationDetector.getDistanceL());
        telemetry.addData("Right Foundation Dist: ", foundationDetector.getDistanceR());
        telemetry.addData("Raw voltage L: ", foundationDetectorInputL.getVoltage());
        telemetry.addData("Raw voltage R: ", foundationDetectorInputR.getVoltage());
        telemetry.update();

        autoModeButton.update(gamepad1.a);

        if (autoModeButton.is(Button.State.DOWN)) {
            isAutoMode = !isAutoMode;
        }

        if (isAutoMode) {
            Pose drivePose = new Pose(0, 0, 0);
            drivePose.r = rPid.getCorrectedOutput(foundationDetector.getDiff());
            if (Math.abs(drivePose.r) < 0.3) {
                drivePose.x = xPid.getCorrectedOutput(blockDetector.getDistanceAvg());
                drivePose.y = yPid.getCorrectedOutput(foundationDetector.getDiff());
            }
            robot.driveTrain.drive(drivePose);
        }
    }
}
