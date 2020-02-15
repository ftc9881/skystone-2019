package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.hardware.motor.OdometryWheel;
import org.firstinspires.ftc.teamcode.hardware.sensor.IDistanceSensor;
import org.firstinspires.ftc.teamcode.hardware.sensor.MaxSonarAnalogSensor;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.teleop.opmodes.drive.BaseDrive;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp(group="Test")
//@Disabled
public class DataTest extends SnapshotTest {

    private OdometryWheel odometryY;
    private Vuforia vuforia;
    private MaxSonarAnalogSensor frontSensor;

    @Override
    protected void initialize() {
        super.initialize();
        odometryY = BatMobile.getInstance().odometryY;
        odometryY.resetEncoder();
        robot.driveTrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        AutoRunner.log("EncoderTestData", String.format("\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s", "SensorY", "VuforiaY", "OdomY", "AverageDriveClicks", "LF", "RF", "RB", "LB"));

        vuforia = Vuforia.createInstance(VisionSystem.CameraType.FRONT_WEBCAM);
        vuforia.startLook(VisionSystem.TargetType.ALL);

        frontSensor = new MaxSonarAnalogSensor(robot.getHardwareMap(), "front sensor");
    }

    @Override
    void onClick() {
        AutoRunner.log("EncoderTestData", String.format("\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s", "sensor?", vuforia.getPose().y, odometryY.getPosition(), robot.driveTrain.getAverageClicks(), robot.driveTrain.lf.getCurrentPosition(), robot.driveTrain.rf.getCurrentPosition(), robot.driveTrain.lb.getCurrentPosition(), robot.driveTrain.rb.getCurrentPosition()));
    }

    @Override
    protected void update() {
        super.update();

        telemetry.addData("sensor (in)", frontSensor.getDistance());
        telemetry.addData("sensor voltage", frontSensor.getDistance());

        telemetry.addData("odometry y", odometryY.getPosition());
        telemetry.addData("lf", robot.driveTrain.lf.getCurrentPosition());
        telemetry.addData("rf", robot.driveTrain.rf.getCurrentPosition());
        telemetry.addData("lb", robot.driveTrain.lb.getCurrentPosition());
        telemetry.addData("rb", robot.driveTrain.rb.getCurrentPosition());

        Pose pose = vuforia.getPose();
        telemetry.addData("Camera", vuforia.getCameraType().name());
        telemetry.addData("X (in)", GeneralMath.round(pose.x, 3));
        telemetry.addData("Y (in)", GeneralMath.round(pose.y, 3));
        telemetry.addData("R (in)", GeneralMath.round(pose.r, 3));
        telemetry.update();
    }

    @Override
    protected void onStop() {
        super.onStop();
        vuforia.stopLook();
    }

}
