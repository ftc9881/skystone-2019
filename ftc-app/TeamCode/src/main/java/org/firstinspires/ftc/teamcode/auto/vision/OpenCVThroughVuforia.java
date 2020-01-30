package org.firstinspires.ftc.teamcode.auto.vision;

import android.content.Context;
import android.graphics.Bitmap;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.SomethingBadHappened;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;
import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.concurrent.BlockingQueue;

public class OpenCVThroughVuforia extends OpenCV {

    VuforiaFrameQueuer queuer;

    public static OpenCVThroughVuforia getInstance() {
        return new OpenCVThroughVuforia();
    }

    private OpenCVThroughVuforia() {
        config = new Configuration("Vision");
    }

    @Override
    protected void startCamera() {
        queuer = new VuforiaFrameQueuer();
        queuer.start();
    }

    @Override
    public void stopLook() {
        queuer.stop();
    }

    class VuforiaFrameQueuer extends Action {

        private VuforiaLocalizer vuforia;
        private BlockingQueue<VuforiaLocalizer.CloseableFrame> queue;

        @Override
        protected void onRun() {
            vuforia = Vuforia.getInstance().vuforiaLocalizer;
            vuforia.setFrameQueueCapacity(1);
            queue = vuforia.getFrameQueue();
        }

        @Override
        protected void insideRun() {
            if (!queue.isEmpty()) {
                Mat mat = new Mat();
                VuforiaLocalizer.CloseableFrame frame;
                try {
                    frame = queue.take();
                } catch (InterruptedException e) {
                    throw new SomethingBadHappened("OpenCVThroughVuforia: error while taking frame from queue");
                }
                Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                Utils.bitmapToMat(bitmap, mat);

                detector.process(mat);
            }
        }

        @Override
        protected boolean runIsComplete() {
            return false;
        }

        @Override
        protected void onEndRun() {
            vuforia.setFrameQueueCapacity(0);
        }
    }


}
