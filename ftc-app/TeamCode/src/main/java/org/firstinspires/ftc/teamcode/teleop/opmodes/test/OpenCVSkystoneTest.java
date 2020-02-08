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
@Disabled
public class OpenCVSkystoneTest extends TeleOpBase {

    OpenCV openCV;
    Button shutterButton = new Button();

    @Override
    protected void initialize() {
        robot.driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        openCV = new OpenCV(config, VisionSystem.CameraType.FRONT_WEBCAM);
        openCV.startLook(VisionSystem.TargetType.SKYSTONE);
    }

    @Override
    protected void update() {
        updateShutter();

        Rect rect = openCV.getFoundRect();
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
