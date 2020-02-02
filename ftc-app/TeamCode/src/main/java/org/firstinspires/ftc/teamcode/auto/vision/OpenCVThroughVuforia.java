package org.firstinspires.ftc.teamcode.auto.vision;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.SomethingBadHappened;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;
import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.concurrent.BlockingQueue;

import static org.firstinspires.ftc.teamcode.auto.vision.VisionSystem.CameraType.FRONT_WEBCAM;

public class OpenCVThroughVuforia extends OpenCV {

    private Vuforia vuforia;
    private VuforiaFrameQueuer queuer;
    private static OpenCVThroughVuforia instance;

    public static OpenCVThroughVuforia createInstance(Command config, CameraType cameraType) {
        instance = new OpenCVThroughVuforia(config, cameraType);
        return instance;
    }

    public static OpenCVThroughVuforia getInstance() {
        return instance;
    }

    private OpenCVThroughVuforia(Command config, CameraType cameraType) {
        this.config = config;
        vuforia = Vuforia.createInstance(cameraType);
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
            Mat mat = convertFrameToMat(frame);
            detector.process(mat);
        }

        @Override
        protected boolean runIsComplete() {
            return false;
        }

        @Override
        protected void onEndRun() {
            vuforiaLocalizer.setFrameQueueCapacity(0);
        }

        private Bitmap convertFrameToBitmap(Frame frame) throws SomethingBadHappened {
            for (int i = 0; i < frame.getNumImages(); i++) {
                Image image = frame.getImage(i);
                if (image.getFormat() == PIXEL_FORMAT.RGB565) {
                    Bitmap bitmap = Bitmap.createBitmap(image.getWidth(), image.getHeight(), Bitmap.Config.RGB_565);
                    bitmap.copyPixelsFromBuffer(image.getPixels());
                    return bitmap;
                }
            }
            throw new SomethingBadHappened("Vuforia frame is not RGB565");
        }

        private Mat convertFrameToMat(Frame frame) throws SomethingBadHappened {
            Bitmap bitmap = convertFrameToBitmap(frame);
            Mat mat = new Mat();
            Utils.bitmapToMat(bitmap, mat);
            return mat;
        }
    }

}
