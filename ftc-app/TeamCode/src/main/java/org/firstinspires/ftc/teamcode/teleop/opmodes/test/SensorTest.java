package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.sensors.SharpDistanceSensor;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp
public class SensorTest extends TeleOpBase {

    SharpDistanceSensor foundationSensorL;
    SharpDistanceSensor foundationSensorR;
    SharpDistanceSensor blockSensorL;
    SharpDistanceSensor blockSensorR;

    Button autoModeButton = new Button();

    PIDController xPid;
    PIDController yPid;
    PIDController rPid;

    boolean isAutoMode = false;

    @Override
    protected void initialize() {

        blockSensorL = new SharpDistanceSensor(hardwareMap, "blockDetectorL", 0.171417, -0.0219884, 0.978115);
        blockSensorR = new SharpDistanceSensor(hardwareMap, "blockDetectorR", 0.195896, -0.0127493, 0.288843);
        foundationSensorL = new SharpDistanceSensor(hardwareMap, "foundationDetectorL", 0.438663, -0.00528896, 0.404581);
        foundationSensorR = new SharpDistanceSensor(hardwareMap, "foundationDetectorR", 0.422507, -0.0545958, 0.461312);

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
//            Pose drivePose = new Pose(0, 0, 0);
//            drivePose.r = rPid.getCorrectedOutput(foundationDetector.getDiff());
//            if (Math.abs(drivePose.r) < 0.3) {
//                drivePose.x = xPid.getCorrectedOutput(blockDetector.getDistanceAvg());
//                drivePose.y = yPid.getCorrectedOutput(foundationDetector.getDistanceAvg());
//            }
//            AutoRunner.log("SensorTestDriveY", drivePose.y);
//            AutoRunner.log("SensorTestDriveR", drivePose.r);
//            telemetry.addData("drive y", drivePose.y);
//            telemetry.addData("drive r", drivePose.r);
//            robot.driveTrain.drive(drivePose);
        }

        telemetry.addData("Auto Align Mode", isAutoMode);
        telemetry.addData("Block Voltage L", blockSensorL.getVoltage());
        telemetry.addData("Block Voltage R", blockSensorR.getVoltage());
        telemetry.addData("Block Distance L", blockSensorL.getDistance());
        telemetry.addData("Block Distance R", blockSensorR.getDistance());
        telemetry.addData("Foundation Voltage L", foundationSensorL.getVoltage());
        telemetry.addData("Foundation Voltage R", foundationSensorR.getVoltage());
        telemetry.addData("Foundation Dist L", foundationSensorL.getDistance());
        telemetry.addData("Foundation Dist R", foundationSensorR.getDistance());
        telemetry.update();
    }
}
