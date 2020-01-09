package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.vision.OpenCV;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp
public class OpenCVTestTeleOp extends TeleOpBase {

    OpenCV openCV;

    @Override
    protected void initialize() {
        openCV = new OpenCV();
        openCV.initialize();
        openCV.startLook(VisionSystem.TargetType.SKYSTONE);
    }

    @Override
    protected void update() {
        telemetry.addData("rect", openCV.detector.foundRectangle().toString());
        telemetry.update();
    }

}
