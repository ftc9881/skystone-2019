package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp
//@Disabled
public class VuforiaTest extends TeleOpBase {

    private Vuforia vuforia;

    @Override
    protected void initialize() {
        vuforia = Vuforia.createInstance(VisionSystem.CameraType.FRONT_WEBCAM);
        vuforia.startLook(VisionSystem.TargetType.ALL);
        robot.driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Initialized Vuforia", "Ready to run");
        telemetry.update();
    }

    @Override
    protected void update() {
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
