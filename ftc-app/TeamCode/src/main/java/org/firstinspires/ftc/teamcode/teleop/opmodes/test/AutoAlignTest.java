package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.hardware.sensor.SharpSensorPair;
import org.firstinspires.ftc.teamcode.teleop.opmodes.drive.BaseDrive;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;

@TeleOp(group="Test")
//@Disabled
public class AutoAlignTest extends BaseDrive {

    private SharpSensorPair foundationSensorPair;
    private SharpSensorPair blockSensorPair;

    private Button autoModeButton = new Button();

    private PIDController xPid;
    private PIDController yPid;
    private PIDController rPid;

    private boolean isAutoMode = false;
    private double deadZone;

    @Override
    protected void initialize() {
        deadZone = config.getDouble("dead zone", 0.1);

        foundationSensorPair = new SharpSensorPair(hardwareMap, "left foundation sensor", "right foundation sensor");
        blockSensorPair = new SharpSensorPair(hardwareMap, "left block sensor", "right block sensor");

        double idealXPosition = config.getDouble("align x target", 0);
        double idealYPosition = config.getDouble("align y target", 0);
        xPid = new PIDController(config, "align x", idealXPosition);
        yPid = new PIDController(config, "align y", idealYPosition);
        rPid = new PIDController(config, "align r", 0);
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
            initAutoAlign();
            isAutoMode = !isAutoMode;
        }
        if (driveInput()) {
            isAutoMode = false;
        }
    }

    private void initAutoAlign() {
        robot.driveTrain.stop();
        xPid.reset();
        yPid.reset();
        rPid.reset();
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
