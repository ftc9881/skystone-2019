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

import java.util.*;

public class CustomFancySkystoneDetector extends DogeCVDetector {
    public DogeCVColorFilter blackFilter = new GrayscaleFilter(0, 25);
    public DogeCVColorFilter yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 100);
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

    private Rect previousBestYellowRect = new Rect();


    public CustomFancySkystoneDetector() {
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
        List<Rect> rectsYellow = new ArrayList<>();

        for (MatOfPoint contour : contoursYellow) {
            rectsYellow.add(Imgproc.boundingRect(contour));
            draw(Imgproc.boundingRect(contour), new Scalar(255, 100, 0));
        }

        List<List<Rect>> yellowBlobs = groupRects(rectsYellow, maxBlobDistance);
        Rect bestYellowRect = previousBestYellowRect;

        for (List<Rect> blob : yellowBlobs) {
            Rect blobBound = boundingRect(blob);
            draw(blobBound, new Scalar(0, 150, 0));

            if (blobBound.area() > bestYellowRect.area()) {
                bestYellowRect = blobBound;
            }
        }
        draw(bestYellowRect, new Scalar(255, 255, 0));

//        List<MatOfPoint> contoursYellow = findContours(yellowFilter, yellowMask);
//        draw(contoursYellow, new Scalar(255, 30, 30));
//
//        List<List<Rect>> yellowBlobs = groupRects(contoursYellow, maxBlobDistance);
//        Rect bestYellowRect = new Rect();
//
//        for (List<Rect> blob : yellowBlobs) {
//            Rect blobBound = boundingRect(blob);
//            draw(blobBound, new Scalar(255, 100, 0));
//
//            if (blobBound.area() > bestYellowRect.area()) {
//                bestYellowRect = blobBound;
//            }
//        }
//


        // BLACK
        List<MatOfPoint> contoursBlack = findContours(blackFilter, blackMask);
        List<Rect> rectsBlack = new ArrayList<>();
        draw(contoursBlack, new Scalar(80, 80, 80));

        for (MatOfPoint contour : contoursBlack) {
            rectsBlack.add(Imgproc.boundingRect(contour));
            draw(Imgproc.boundingRect(contour), new Scalar(0, 0, 255));
        }

        rectsBlack = filterByBound(rectsBlack, bestYellowRect);

        //filter black contours
        List<List<Rect>> blackBlobs = groupRects(rectsBlack, maxBlobDistance);
        Rect bestBlobRect = foundRect;

        for (List<Rect> blob : blackBlobs) {
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
            previousBestYellowRect = bestYellowRect;
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

    private List<List<Rect>> groupRects(List<Rect> rects, int maxBlobDistance) {
        List<List<Rect>> toReturn = new ArrayList<>();
        List<Rect> unusedRects = new ArrayList<>();
        unusedRects.addAll(rects);

        int blobIndex = 0;
        while (!unusedRects.isEmpty()) {
            LinkedList<Rect> toProcess = new LinkedList<>();
            toProcess.add(unusedRects.remove(0));
            toReturn.add(new ArrayList<Rect>());
            while (!toProcess.isEmpty()) {
                Rect currentRect = toProcess.poll();
                for (int i = 0; i < unusedRects.size(); i++) {
                    if (distance(getCenterPoint(currentRect), getCenterPoint(unusedRects.get(i))) < maxBlobDistance) {
                        toProcess.add(unusedRects.get(i));
                        toReturn.get(blobIndex).add(unusedRects.remove(i));
                        i--;
                    }
                }
            }
            blobIndex++;
            //run algorithm on firstNode
        }

        return toReturn;
    }

    private List<Rect> filterByBound(List<Rect> rects, Rect boundingRect) {
        List<Rect> toReturn = new ArrayList<>();

        for (Rect rect : rects) {
            if (boundingRect.contains(getCenterPoint(rect))) {
                toReturn.add(rect);
            }
        }
       return toReturn;
    }
}