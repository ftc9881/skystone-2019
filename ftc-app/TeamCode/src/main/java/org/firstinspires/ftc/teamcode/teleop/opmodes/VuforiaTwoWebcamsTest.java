package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp
@Disabled
public class VuforiaTwoWebcamsTest extends TeleOpBase {

    private Vuforia vuforia;
    private Button webcam1Button = new Button();
    private Button webcam2Button = new Button();

    @Override
    protected void initialize() {
        vuforia = Vuforia.createInstance(hardwareMap, VisionSystem.CameraType.BACK_WEBCAM);
        vuforia.startLook(VisionSystem.TargetType.ALL);

        telemetry.addData("Initialized Vuforia", "Ready to run");
        telemetry.update();
    }

    @Override
    protected void update() {
        webcam1Button.update(gamepad1.a);
        webcam2Button.update(gamepad1.b);

        if (webcam1Button.is(Button.State.DOWN)) {
            vuforia.setActiveCamera(VisionSystem.CameraType.FRONT_WEBCAM);
        }
        if (webcam2Button.is(Button.State.DOWN)) {
            vuforia.setActiveCamera(VisionSystem.CameraType.BACK_WEBCAM);
        }

        Pose pose = vuforia.getPose();
        telemetry.addData("Camera", vuforia.getCameraType().name());
        telemetry.addData("X (in)", GeneralMath.round(pose.x, 3));
        telemetry.addData("Y (in)", GeneralMath.round(pose.y, 3));
        telemetry.addData("R (in)", GeneralMath.round(pose.r, 3));
        telemetry.update();
    }

    private void toggleCamera() {
        VisionSystem.CameraType currentCamera = vuforia.getCameraType();
        VisionSystem.CameraType cameraType = currentCamera == VisionSystem.CameraType.FRONT_WEBCAM ? VisionSystem.CameraType.BACK_WEBCAM : VisionSystem.CameraType.FRONT_WEBCAM;
        vuforia.setActiveCamera(cameraType);
    }
    
    @Override
    protected void onStop() {
        super.onStop();
        vuforia.stopLook();
    }
}
