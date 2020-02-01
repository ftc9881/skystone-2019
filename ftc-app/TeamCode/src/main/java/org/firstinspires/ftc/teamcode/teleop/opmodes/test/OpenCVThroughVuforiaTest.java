package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.vision.OpenCVThroughVuforia;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;
import org.opencv.core.Rect;

import static org.firstinspires.ftc.teamcode.auto.vision.VisionSystem.CameraType.FRONT_WEBCAM;

@TeleOp
//@Disabled
public class OpenCVThroughVuforiaTest extends TeleOpBase {

    Vuforia vuforia;
    OpenCVThroughVuforia openCV;
    Button shutterButton = new Button();

    @Override
    protected void initialize() {
        robot.driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Configuration config = new Configuration("OpenCVThroughVuforia");
        openCV = OpenCVThroughVuforia.createInstance(config, hardwareMap, FRONT_WEBCAM);
        openCV.startLook(VisionSystem.TargetType.SKYSTONE);

        vuforia = Vuforia.getInstance();
    }

    @Override
    protected void update() {
        updateShutter();

        Rect rect = openCV.detector.foundRectangle();
        telemetry.addData("Rect", rect.toString());
        telemetry.addData("CenterX", rect.x + rect.width / 2);

        Pose pose = vuforia.getPose();
        telemetry.addData("Camera", vuforia.getCameraType().name());
        telemetry.addData("X (in)", GeneralMath.round(pose.x, 3));
        telemetry.addData("Y (in)", GeneralMath.round(pose.y, 3));
        telemetry.addData("R (in)", GeneralMath.round(pose.r, 3));

        telemetry.update();
    }

    private void updateShutter() {
        shutterButton.update(gamepad1.right_bumper || gamepad1.left_bumper);
        if (shutterButton.is(Button.State.DOWN)) {
            openCV.writeCurrentImage();
        }
    }
}
