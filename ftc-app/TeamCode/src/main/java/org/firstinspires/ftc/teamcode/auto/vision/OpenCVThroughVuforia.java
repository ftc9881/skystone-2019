package org.firstinspires.ftc.teamcode.auto.vision;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.SomethingBadHappened;
import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.concurrent.BlockingQueue;

import static org.firstinspires.ftc.teamcode.auto.vision.VisionSystem.CameraType.FRONT_WEBCAM;

public class OpenCVThroughVuforia extends OpenCV {

    private Vuforia vuforia;
    private VuforiaFrameQueuer queuer;

    public static OpenCVThroughVuforia createInstance(Configuration config, HardwareMap hardwareMap, CameraType cameraType) {
        return new OpenCVThroughVuforia(config, hardwareMap, cameraType);
    }

    private OpenCVThroughVuforia(Configuration config, HardwareMap hardwareMap, CameraType cameraType) {
        this.config = config;
        vuforia = Vuforia.createInstance(hardwareMap, cameraType);
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

        private VuforiaLocalizer vuforiaLocalizer;
        private BlockingQueue<VuforiaLocalizer.CloseableFrame> queue;

        @Override
        protected void onRun() {
            vuforiaLocalizer = vuforia.vuforiaLocalizer;
            vuforiaLocalizer.enableConvertFrameToBitmap();
            vuforiaLocalizer.setFrameQueueCapacity(1);
            queue = vuforiaLocalizer.getFrameQueue();
        }

        @Override
        protected void insideRun() throws SomethingBadHappened {
            VuforiaLocalizer.CloseableFrame frame;
            try {
                frame = queue.take();
            } catch (InterruptedException e) {
                throw new SomethingBadHappened("OpenCVThroughVuforia: error while taking frame from queue");
            }

            if (frame != null) {
                Bitmap bitmap = vuforiaLocalizer.convertFrameToBitmap(frame);
                Mat mat = new Mat();
                Utils.bitmapToMat(bitmap, mat);
                detector.process(mat);
            }
            else {
                AutoRunner.log(":(");
            }
        }

        @Override
        protected boolean runIsComplete() {
            return false;
        }

        @Override
        protected void onEndRun() {
            vuforiaLocalizer.setFrameQueueCapacity(0);
        }
    }


}
