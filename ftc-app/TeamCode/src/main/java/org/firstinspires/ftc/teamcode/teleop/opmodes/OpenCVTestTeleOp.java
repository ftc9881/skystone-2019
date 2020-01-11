package org.firstinspires.ftc.teamcode.teleop.opmodes;

import android.os.Environment;

import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.vision.OpenCV;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.text.SimpleDateFormat;
import java.util.Date;

@TeleOp
//@Disabled
public class OpenCVTestTeleOp extends TeleOpBase {

    OpenCV openCV;
    Button shutterButton = new Button();

    @Override
    protected void initialize() {
        openCV = new OpenCV();
        openCV.initialize();
        openCV.startLook(VisionSystem.TargetType.SKYSTONE);
    }

    @Override
    protected void update() {
        updateShutter();

        telemetry.addData("Rect", openCV.detector.foundRectangle().toString());
        telemetry.update();
    }

    private void updateShutter() {
        shutterButton.update(gamepad1.right_bumper || gamepad1.left_bumper);
        if (shutterButton.is(Button.State.DOWN)) {
            saveCurrentImage();
        }
    }

    private void saveCurrentImage() {
        Mat mat = openCV.detector.getRenderMat(1);
        Mat newMat = new Mat();
        SimpleDateFormat formatter = new SimpleDateFormat("MM-dd-yyyy_HH:mm:ss.SSS");
        Date date = new Date(System.currentTimeMillis());
        String path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES) + "/" + formatter.format(date) + ".png";
        Imgproc.cvtColor(mat, newMat, Imgproc.COLOR_BGR2RGB);
        boolean success = Imgcodecs.imwrite(path, newMat);
        AutoRunner.log("OpenCV", success);
        AutoRunner.log("OpenCV", "Wrote image " + path);
    }

}
