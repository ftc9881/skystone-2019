package org.firstinspires.ftc.teamcode.auto.vision;

import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.GrayscaleFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class CustomSkystoneDetector extends DogeCVDetector {
    public DogeCVColorFilter blackFilter = new GrayscaleFilter(0, 25);
    public DogeCVColorFilter yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 70);
    // White for home testing
//    public DogeCVColorFilter yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.WHITE, 180);

    public Rect lookWindow;
    public int maxBlobDistance;

    private Rect foundRect = new Rect();

    private Mat rawImage = new Mat();
    private Mat workingMat = new Mat();
    private Mat displayMat = new Mat();
    private Mat blackMask = new Mat();
    private Mat yellowMask = new Mat();
    private Mat hierarchy  = new Mat();


    public CustomSkystoneDetector() {
        detectorName = "Skystone Detector";
    }
    public Rect foundRectangle() {
        return foundRect;
    }


    @Override
    public Mat process(Mat input) {
        input.copyTo(rawImage);
        input.copyTo(workingMat);
        input.copyTo(displayMat);
        input.copyTo(blackMask);
        lookWindow = lookWindow == null ? OpenCV.CAMERA_RECT : lookWindow;

        // YELLOW

        List<MatOfPoint> contoursYellow = findContours(yellowFilter, yellowMask);
        draw(contoursYellow, new Scalar(255, 30, 30));

        Rect bestYellowRect = new Rect();

        for (MatOfPoint cont : contoursYellow) {
            Rect rect = Imgproc.boundingRect(cont);
            draw(rect, new Scalar(255,100,0));

            if (rect.area() > bestYellowRect.area()) {
                bestYellowRect = rect;
            }
        }

        draw(bestYellowRect, new Scalar(255, 255, 0));


        // BLACK

        List<MatOfPoint> contoursBlack = findContours(blackFilter, blackMask);
        // Java 8 sort
//        contoursBlack.sort( (MatOfPoint a, MatOfPoint b) -> Imgproc.boundingRect(a).x - Imgproc.boundingRect(b).x);
        // Java 7 sort
        Collections.sort(contoursBlack, new Comparator<MatOfPoint>() {
            @Override
            public int compare(MatOfPoint o1, MatOfPoint o2) {
                return Imgproc.boundingRect(o1).x - Imgproc.boundingRect(o2).x;
            }
        });

        draw(contoursBlack, new Scalar(80, 80, 80));

        List<List<Rect>> blobsNearEachOther = new ArrayList<>();
        List<Rect> currentBlob = new ArrayList<>();
        Rect previousContourRect = new Rect();

        for (MatOfPoint contour : contoursBlack) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            Point center = getCenterPoint(boundingRect);

            if (bestYellowRect.contains(center)) {
                boolean contourIsFar = distance(center, getCenterPoint(previousContourRect)) > maxBlobDistance;
                if (contourIsFar) {
                    blobsNearEachOther.add(currentBlob);
                    currentBlob.clear();
                }
                currentBlob.add(boundingRect);
                previousContourRect = boundingRect;

                draw(center, new Scalar(0, 0, 255));
            }

            draw(boundingRect, new Scalar(0, 0, 200));
        }
        blobsNearEachOther.add(currentBlob);


        Rect bestBlobRect = new Rect();

        for (List<Rect> blob : blobsNearEachOther) {
            Rect blobBound = boundingRect(blob);
            Point blobCenter = getCenterPoint(blobBound);
            draw(blobBound, new Scalar(0, 150, 0));

            if (blobBound.area() > bestBlobRect.area() && bestYellowRect.contains(blobCenter)) {
                bestBlobRect = blobBound;
            }
        }


        // WINDOW

        Rect bestRect = bestBlobRect;
        Point bestRectCenter = getCenterPoint(bestRect);
        draw(lookWindow, new Scalar(255, 255, 255));
        draw(bestRectCenter, new Scalar(0, 255, 0));

        found = bestRect.area() > 0 && lookWindow.contains(bestRectCenter);

        if (found) {
            draw(bestRect, new Scalar(0, 255, 0));
            foundRect = bestRect;
        }


        // RENDER

        switch (stageToRenderToViewport) {
            case THRESHOLD: {
                Imgproc.cvtColor(blackMask, blackMask, Imgproc.COLOR_GRAY2BGR);

                return blackMask;
            }
            case RAW_IMAGE: {
                return rawImage;
            }
            default: {
                return displayMat;
            }
        }
    }


    @Override
    public void useDefaults() {
        lookWindow = OpenCV.CAMERA_RECT;
        maxBlobDistance = 50;
    }

    private Rect boundingRect(List<Rect> rects) {
        int minX = 999;
        int minY = 999;
        int maxX = 0;
        int maxY = 0;
        for (Rect rect : rects) {
            if (rect.x < minX) {
                minX = rect.x;
            }
            if (rect.y < minY) {
                minY = rect.y;
            }
            if (rect.x + rect.width > maxX) {
                maxX = rect.x + rect.width;
            }
            if (rect.y + rect.height > maxY) {
                maxY = rect.y + rect.height;
            }
        }

        return new Rect(minX, minY, maxX - minX, maxY - minY);
    }

    private double distance(Point a, Point b) {
        return Math.sqrt(Math.pow(b.x - a.x, 2) + Math.pow(b.y - a.y, 2));
    }

    private Point getCenterPoint(Rect rect) {
        return new Point(rect.x + rect.width/2, rect.y + rect.height/2);
    }

    private List<MatOfPoint> findContours(DogeCVColorFilter filter, Mat mask) {
        filter.process(workingMat.clone(), mask);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        return contours;
    }


    private void draw(Rect rect, Scalar color) {
        Imgproc.rectangle(displayMat, rect.tl(), rect.br(), color, 2);
    }

    private void draw(Point point, Scalar color) {
        Imgproc.circle(displayMat, point, 2, color);
    }

    private void draw(List<MatOfPoint> contours, Scalar color) {
        // Not draw contours for now, since can look messy
//        Imgproc.drawContours(displayMat, contours, -1, color, 2);
    }

}