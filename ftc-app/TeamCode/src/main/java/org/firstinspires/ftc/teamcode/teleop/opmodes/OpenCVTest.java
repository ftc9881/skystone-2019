package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.vision.OpenCV;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;
import org.opencv.core.Rect;

@TeleOp
//@Disabled
public class OpenCVTest extends TeleOpBase {

    OpenCV openCV;
    Button shutterButton = new Button();

    @Override
    protected void initialize() {
        openCV = new OpenCV();
        openCV.startLook(VisionSystem.TargetType.SKYSTONE);

        Rect cropRect = new Rect();
        cropRect.x = config.getInt("crop x", 0);
        cropRect.y = config.getInt("crop y", 0);
        cropRect.width = config.getInt("crop w", 0);
        cropRect.height= config.getInt("crop h", 0);
        openCV.detector.cropRect = cropRect;
        openCV.detector.blobDistanceThreshold = config.getInt("skystone blob distance", 50);
        openCV.detector.minimumArea = config.getInt("skystone min area", 0);
    }

    @Override
    protected void update() {
        updateShutter();

        Rect rect = openCV.detector.foundRectangle();
        telemetry.addData("Rect", rect.toString());
        telemetry.addData("CenterX", rect.x + rect.width / 2);
        telemetry.update();
    }

    private void updateShutter() {
        shutterButton.update(gamepad1.right_bumper || gamepad1.left_bumper);
        if (shutterButton.is(Button.State.DOWN)) {
            openCV.writeCurrentImage();
        }
    }
}
