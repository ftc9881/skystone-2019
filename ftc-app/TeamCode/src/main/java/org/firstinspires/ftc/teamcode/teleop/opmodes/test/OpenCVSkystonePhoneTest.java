package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.vision.OpenCV;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.opencv.core.Rect;

@TeleOp(group="Test")
@Disabled
public class OpenCVSkystonePhoneTest extends SnapshotTest {

    private OpenCV openCV;

    @Override
    protected void initialize() {
        super.initialize();
        openCV = new OpenCV(config, VisionSystem.CameraType.PHONE);
        openCV.startLook(VisionSystem.TargetType.SKYSTONE);
    }

    @Override
    protected void update() {
        super.update();
        Rect rect = openCV.getFoundRect();
        telemetry.addData("Rect", rect.toString());
        telemetry.addData("CenterX", rect.x + rect.width / 2);
        telemetry.update();
    }

    @Override
    void onClick() {
        openCV.writeCurrentImage();
    }

}
