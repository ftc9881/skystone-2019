package org.firstinspires.ftc.teamcode.auto.vision;

import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.math.Line;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.math.GeneralMath.getCenterPoint;

public class VumarkTemplateDetector extends OpenCVDetector {
    public DogeCVColorFilter whiteFilter;
    public int minVumarkWidth;
    public int lineThreshold;
    public int minLineLength;
    public int maxLineGap;
    public int maxLineWidth;

    private Mat whiteMask = new Mat();

    public VumarkTemplateDetector() { }

    @Override
    public Mat process(Mat input) {
        input.copyTo(whiteMask);

        List<MatOfPoint> contoursWhite = findContours(whiteFilter, whiteMask);
        draw(contoursWhite, new Scalar(255, 255, 255));
        Imgproc.cvtColor(whiteMask, thresholdMat, Imgproc.COLOR_GRAY2BGR);
        List<Line> lines = detectLines(thresholdMat);
        Line vumarkDiagonal = getDiagonal(lines);
        draw(vumarkDiagonal, new Scalar(0, 200, 0));

        found = vumarkDiagonal.width() > minVumarkWidth;
        if (found) {
            foundRect = new Rect(vumarkDiagonal.point1, vumarkDiagonal.point2);
            draw(foundRect, new Scalar(0, 255, 0));
        }

        // RENDER
        return getMat(stageToRenderToViewport);
    }

    @Override
    public void setConfig(Command config) {
        whiteFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.WHITE);
        minVumarkWidth = config.getInt("min vumark width", 100);
        lineThreshold= config.getInt("line threshold", 50);
        minLineLength = config.getInt("min line length", 50);
        maxLineGap = config.getInt("max line gap", 20);
        maxLineWidth = config.getInt("max line width", 10);
    }

    @Override
    public Pose getPose() {
        // TODO: Regresison function mapping width to y distance
        return new Pose(
            foundRect.x - (OpenCV.CAMERA_RECT.x + OpenCV.CAMERA_RECT.width / 2),
            foundRect.width,
            0
        );
    }

    private List<MatOfPoint> findContours(DogeCVColorFilter filter, Mat mask) {
        filter.process(workingMat.clone(), mask);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        return contours;
    }

    private List<Line> detectLines(Mat src) {
        Mat dst = new Mat();
        Mat cdst = new Mat();

//        Imgproc.Canny(src, dst, 50, 200, 3, false);
        Imgproc.Canny(src, dst, 0, 255, Imgproc.THRESH_BINARY | Imgproc.THRESH_OTSU);
        Imgproc.cvtColor(dst, cdst, Imgproc.COLOR_GRAY2BGR);
        Mat cdstP = cdst.clone();

        Mat linesP = new Mat();
        Imgproc.HoughLinesP(dst, linesP, 1, Math.PI/180, lineThreshold, minLineLength, maxLineGap);

        List<Line> lines = new ArrayList<>();
        for (int x = 0; x < linesP.rows(); x++) {
            double[] l = linesP.get(x, 0);
            Line line = new Line(l);

            draw(cdstP, line, new Scalar(80, 80, 255));
            if (line.width() < maxLineWidth) {
                lines.add(line);
            }
        }
        debugMat = cdstP;

        return lines;
    }

    private Line getDiagonal(List<Line> lines) {
        Line leftMost = new Line();
        Line rightMost = new Line();

        for (Line line : lines) {
            if (line.minX() < leftMost.minX()) {
                leftMost = line;
            } else if (line.maxX() > rightMost.maxX()) {
                rightMost = line;
            }
        }
        return new Line(
                new Point(leftMost.minX(), leftMost.maxY()),
                new Point(rightMost.maxX(), rightMost.minY()) );
    }

}