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
@Disabled
public class SensorAlignTest extends TeleOpBase {

    SharpPair analogBlockDetector;

    Button autoModeButton = new Button();

    PIDController xPid;
    PIDController yPid;
    PIDController rPid;

    boolean isAutoMode;

    @Override
    protected void initialize() {

        // block detection
        AnalogInput bdinput_L = robot.hardwareMap.analogInput.get("bdL");
        AnalogInput bdinput_R = robot.hardwareMap.analogInput.get("bdR");
        SharpDistanceSensor bsensor_left = new SharpDistanceSensor(bdinput_L);
        SharpDistanceSensor bsensor_right = new SharpDistanceSensor(bdinput_R);

        // foundation detection
        AnalogInput fdinput_L = robot.hardwareMap.analogInput.get("fdL");
        AnalogInput fdinput_R = robot.hardwareMap.analogInput.get("fdR");
        SharpDistanceSensor fsensor_left = new SharpDistanceSensor(fdinput_L);
        SharpDistanceSensor fsensor_right = new SharpDistanceSensor(fdinput_R);

         SharpPair blockDetector = new SharpPair(bsensor_left, bsensor_right,
             config.getDouble("blockDetectDist", 21),
             config.getDouble("blockDetectMargin", 2));

         SharpPair foundationDetector = new SharpPair(fsensor_left, fsensor_right,
             config.getDouble("foundationDetectDist", 10),
             config.getDouble("foundationDetectMargin", 2));

        double idealYPosition = config.getDouble("sensor y target", 0);
        xPid = new PIDController(config, "sensor x", 0);
        yPid = new PIDController(config, "sensor y", idealYPosition);
        rPid = new PIDController(config, "sensor r", 0);
    }

    @Override
    protected void update() {
        autoModeButton.update(gamepad1.a);

        if (autoModeButton.is(Button.State.DOWN)) {
            isAutoMode = !isAutoMode;
        }

        if (isAutoMode) {
            // TODO: auto align stuff

            Pose drivePose = new Pose(0, 0, 0);
            drivePose.x = xPid.getCorrectedOutput(analogBlockDetector.getDiff());
            drivePose.x = xPid.getCorrectedOutput(analogBlockDetector.getDiff());

            robot.driveTrain.drive(drivePose);
        }

    }
}
