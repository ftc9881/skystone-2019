package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.vision.OpenCV;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;
import org.opencv.core.Rect;

@TeleOp(group="Test")
//@Disabled
public class OpenCVSkystoneTest extends SnapshotTest {

    private OpenCV openCV;

    @Override
    protected void initialize() {
        super.initialize();
        openCV = new OpenCV(config, VisionSystem.CameraType.FRONT_WEBCAM);
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
