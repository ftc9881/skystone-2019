package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.hardware.sensor.SharpSensorPair;
import org.firstinspires.ftc.teamcode.teleop.opmodes.drive.BaseDrive;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;

@TeleOp(group="Test")
public class AutoAlignTest extends BaseDrive {

    private SharpSensorPair foundationSensorPair;
    private SharpSensorPair blockSensorPair;

    private Button autoModeButton = new Button();

    private PIDController xPid;
    private PIDController yPid;
    private PIDController rPid;

    boolean isAutoMode = false;
    double deadZone;

    @Override
    protected void initialize() {
        /*
        TODO: put into HardwareConstants.json
        "left block sensor a": "0.171417",
        "left block sensor b": "-0.0219884",
        "left block sensor c": "0.978115",
        "right block sensor a": "0.195896",
        "right block sensor b": "-0.0127493",
        "right block sensor c": "0.288843",
        "left foundation sensor a": "0.438663",
        "left foundation sensor b": "-0.00528896",
        "left foundation sensor c": "0.404581"
        "right foundation sensor a": "0.422507",
        "right foundation sensor b": "-0.0545958",
        "right foundation sensor c": "0.461312"
         */
        deadZone = config.getDouble("dead zone", 0.1);

        foundationSensorPair = new SharpSensorPair(hardwareMap, "left foundation sensor", "right foundation sensor");
        blockSensorPair = new SharpSensorPair(hardwareMap, "left block sensor", "right block sensor");

        double idealXPosition = config.getDouble("sensor x target", 0);
        xPid = new PIDController(config, "sensor x", idealXPosition);
        yPid = new PIDController(config, "sensor y", 25);
        rPid = new PIDController(config, "sensor r", 0);
    }

    @Override
    protected void update() {
        updateAutoMode();
        if (isAutoMode) {
            doAutoAlign();
        } else {
            doManualDrive();
        }
        updateTelemetry();
    }

    private void updateAutoMode() {
        autoModeButton.update(gamepad1.a);
        if (autoModeButton.is(Button.State.DOWN)) {
            robot.driveTrain.stop();
            isAutoMode = !isAutoMode;
        }
        if (driveInput()) {
            isAutoMode = false;
        }
    }

    private void doAutoAlign() {
        Pose drivePose = new Pose(0, 0, 0);
        drivePose.r = rPid.getCorrectedOutput(foundationSensorPair.getDifference());
        if (Math.abs(drivePose.r) < 0.3) {
            drivePose.x = xPid.getCorrectedOutput(blockSensorPair.getAverage());
            drivePose.y = yPid.getCorrectedOutput(foundationSensorPair.getAverage());
        }
        AutoRunner.log("SensorTestDriveY", drivePose.y);
        AutoRunner.log("SensorTestDriveR", drivePose.r);
        telemetry.addData("drive y", drivePose.y);
        telemetry.addData("drive r", drivePose.r);
        robot.driveTrain.drive(drivePose);
    }

    private void doManualDrive() {
        super.update();
    }

    private void updateTelemetry() {
        telemetry.addData("Auto Align Mode", isAutoMode);
        telemetry.addData("Block Voltage L", blockSensorPair.left.getVoltage());
        telemetry.addData("Block Voltage R", blockSensorPair.right.getVoltage());
        telemetry.addData("Block Distance L", blockSensorPair.left.getDistance());
        telemetry.addData("Block Distance R", blockSensorPair.right.getDistance());
        telemetry.addData("Foundation Voltage L", foundationSensorPair.left.getVoltage());
        telemetry.addData("Foundation Voltage R", foundationSensorPair.right.getVoltage());
        telemetry.addData("Foundation Dist L", foundationSensorPair.left.getDistance());
        telemetry.addData("Foundation Dist R", foundationSensorPair.right.getDistance());
        telemetry.update();
    }

    private boolean driveInput() {
        return isInputting(gamepad1.left_stick_x) || isInputting(gamepad1.left_stick_y) || isInputting(gamepad1.right_stick_x);
    }

    private boolean isInputting(double input) {
        return Math.abs(input) > deadZone;
    }
}
