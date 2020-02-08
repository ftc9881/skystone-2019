package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.math.Line;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

/**
 * Modified DogeCVDetector.
 */
public abstract class OpenCVDetector extends OpenCvPipeline {

    public enum Stage {
        DISPLAY,
        THRESHOLD,
        DEBUG,
        RAW_IMAGE
    }

    public abstract Mat process(Mat input);
    public abstract void setConfig(Command config);
    public abstract Pose getPose();

    protected boolean found = false;
    protected boolean flipImage = false;
    protected Rect foundRect = new Rect();

    protected Mat rawImage = new Mat();
    protected Mat displayMat = new Mat();
    protected Mat debugMat = new Mat();
    protected Mat thresholdMat = new Mat();
    protected Mat workingMat = new Mat();
    protected Mat hierarchy  = new Mat();

    protected Stage stageToRenderToViewport = Stage.DISPLAY;
    private Stage[] stages = Stage.values();

    public Rect getFoundRect() {
        return foundRect;
    }

    public Mat getMat(Stage stage) {
        switch (stage) {
            case THRESHOLD:
                return thresholdMat;
            case RAW_IMAGE:
                return rawImage;
            case DEBUG:
                return debugMat;
            default:
            case DISPLAY:
                return displayMat;
        }
    }

    @Override
    public final Mat processFrame(Mat input) {

        if (input.empty()) {
            return input;
        }

        if (flipImage) {
            Core.rotate(input, input, Core.ROTATE_180);
        }

        input.copyTo(rawImage);
        input.copyTo(displayMat);
        input.copyTo(workingMat);
        input.copyTo(hierarchy);

        return process(input);
    }

    @Override
    public void onViewportTapped()
    {
        int current = stageToRenderToViewport.ordinal();
        int next = current + 1 >= stages.length ? 0 : current + 1;
        stageToRenderToViewport = stages[next];
    }

    void draw(Rect rect, Scalar color) {
        Imgproc.rectangle(displayMat, rect.tl(), rect.br(), color, 2);
    }

    void draw(Point point, Scalar color) {
        Imgproc.circle(displayMat, point, 2, color);
    }

    void draw(List<MatOfPoint> contours, Scalar color) {
        Imgproc.drawContours(displayMat, contours, -1, color, 2);
    }

    void draw(Line line, Scalar color) {
        Imgproc.line(displayMat, line.point1, line.point2, color, 2, Imgproc.LINE_AA, 0);
    }

    void draw(Mat mat, Line line, Scalar color) {
        Imgproc.line(mat, line.point1, line.point2, color, 2, Imgproc.LINE_AA, 0);
    }


}
