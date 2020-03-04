package org.firstinspires.ftc.teamcode.auto.vision;

import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.GrayscaleFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.LinkedList;

import static org.firstinspires.ftc.teamcode.math.GeneralMath.distance;
import static org.firstinspires.ftc.teamcode.math.GeneralMath.getCenterPoint;

public class SkystoneDetector extends OpenCVDetector {
    public DogeCVColorFilter blackFilter;
    public DogeCVColorFilter yellowFilter;
    public int yellowBlobbingThreshold;
    public int blackBlobbingThreshold;
    public int minimumArea;
    public Rect cropRect = new Rect();

    private Mat blackMask = new Mat();
    private Mat yellowMask = new Mat();

    public SkystoneDetector() { }

    @Override
    public Mat process(Mat input) {
        Core.rotate(input, input, Core.ROTATE_180);
        if (cropRect.width > 0 && cropRect.height > 0) {
            input = input.submat(cropRect);
        }
        input.copyTo(rawImage);
        input.copyTo(workingMat);
        input.copyTo(displayMat);
        input.copyTo(thresholdMat);
        input.copyTo(blackMask);
        input.copyTo(yellowMask);

        List<MatOfPoint> contoursYellow = findContours(yellowFilter, yellowMask);
        List<Rect> rectsYellow = contoursToRects(contoursYellow);
        List<List<Rect>> listOfYellowBlobs = groupIntoBlobs(rectsYellow, yellowBlobbingThreshold);
        Rect yellowBoundingRect = chooseBestYellow(listOfYellowBlobs);

        List<MatOfPoint> contoursBlack = findContours(blackFilter, blackMask);
        List<Rect> rectsBlack = contoursToRects(contoursBlack);
//        List<Rect> rectsBlackInYellow = filterByBound(rectsBlack, yellowBoundingRect);
//        List<List<Rect>> listOfBlackBlobs = groupIntoBlobs(rectsBlackInYellow, blackBlobbingThreshold);
        List<List<Rect>> listOfBlackBlobs = groupIntoBlobs(rectsBlack, blackBlobbingThreshold);
        Rect bestSkystoneRect = chooseBestBlack(listOfBlackBlobs);

        draw(contoursYellow, new Scalar(255, 150, 0));
        draw(contoursBlack, new Scalar(80, 80, 80));
        draw(yellowBoundingRect, new Scalar(255, 255, 0));
        draw(bestSkystoneRect, new Scalar(0, 255, 0));
        draw(getCenterPoint(bestSkystoneRect), new Scalar(0, 255, 0));

        found = bestSkystoneRect.area() > 0;
        if (found && bestSkystoneRect.area() > foundRect.area()) {
            foundRect = bestSkystoneRect;
        }

        // RENDER
        Imgproc.cvtColor(blackMask, thresholdMat, Imgproc.COLOR_GRAY2BGR);
        return getMat(stageToRenderToViewport);
    }

    @Override
    public void setConfig(Command config) {
        blackFilter = new GrayscaleFilter(0, 50);
        yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 90);
        yellowBlobbingThreshold = config.getInt("yellow blobbing", 100);
        blackBlobbingThreshold = config.getInt("black blobbing", 50);
        minimumArea = config.getInt("min area", 100);

        String sideKey = AutoRunner.getSide().getKey();
        cropRect.x = config.getInt(sideKey + " crop x", 0);
        cropRect.y = config.getInt(sideKey + " crop y", 0);
        cropRect.width = config.getInt(sideKey + " crop w", 0);
        cropRect.height= config.getInt(sideKey + " crop h", 0);
    }

    @Override
    public Pose getPose() {
        // TODO: pose for skystone; although probably won't need
        return new Pose();
    }

    private Rect boundingRect(List<Rect> rects) {
        int minX = 999;
        int minY = 999;
        int maxX = 0;
        int maxY = 0;
        for (Rect rect : rects) {
            minX = Math.min(rect.x, minX);
            minY = Math.min(rect.y, minY);
            maxX = Math.max(rect.x + rect.width, maxX);
            maxY = Math.max(rect.y + rect.height, maxY);
        }

        return new Rect(minX, minY, maxX - minX, maxY - minY);
    }

    private List<MatOfPoint> findContours(DogeCVColorFilter filter, Mat mask) {
        filter.process(workingMat.clone(), mask);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        return contours;
    }

   private Rect chooseBestYellow(List<List<Rect>> listOfYellowBlobs) {
       Rect bestYellowRect = new Rect();
       for (List<Rect> blob : listOfYellowBlobs) {
           Rect blobBound = boundingRect(blob);
           draw(blobBound, new Scalar(255 , 100, 0));

           if (blobBound.area() > bestYellowRect.area()) {
               bestYellowRect = blobBound;
           }
       }
       return bestYellowRect;
   }

   private Rect chooseBestBlack(List<List<Rect>> listOfBlackBlobs) {
        if (listOfBlackBlobs.size() == 1) {
            return boundingRect(listOfBlackBlobs.get(0));
        }
        Rect bestBlackRect = new Rect();
       for (List<Rect> blob : listOfBlackBlobs) {
           Rect blobBound = boundingRect(blob);
           draw(blobBound, new Scalar(0, 150, 0));
//           if (blobBound.y > bestBlackRect.y && blobBound.area() > minimumArea) {
           if (blobBound.area() > bestBlackRect.area()) {
               bestBlackRect = blobBound;
           }
       }
       return bestBlackRect;
   }

   private  List<Rect> contoursToRects(List<MatOfPoint> contours) {
       List<Rect> rects = new ArrayList<>();
       for (MatOfPoint contour : contours) {
           rects.add(Imgproc.boundingRect(contour));
       }
       return rects;
    }

   private List<List<Rect>> groupIntoBlobs(List<Rect> rects, int blobDistanceThreshold) {
       List<List<Rect>> listOfBlobs = new ArrayList<>();
        List<Rect> unusedRects = new ArrayList<>(rects);

       while (!unusedRects.isEmpty()) {
           LinkedList<Rect> toProcess = new LinkedList<>();
           toProcess.add(unusedRects.remove(0));
           List<Rect> currentBlob = new ArrayList<>();
           while (!toProcess.isEmpty()) {
               Rect currentRect = toProcess.poll();
               currentBlob.add(currentRect);

               for (int i = 0; i < unusedRects.size(); i++) {
                   if (distance(getCenterPoint(currentRect), getCenterPoint(unusedRects.get(i))) < blobDistanceThreshold) {
                       toProcess.add(unusedRects.remove(i));
                       i--;
                   }
               }
           }
           listOfBlobs.add(currentBlob);
       }

       return listOfBlobs;
   }

   private List<Rect> filterByBound(List<Rect> rects, Rect boundingRect) {
       List<Rect> rectsInsideBound = new ArrayList<>();
       for (Rect rect : rects) {
           if (boundingRect.contains(getCenterPoint(rect))) {
               rectsInsideBound.add(rect);
           }
       }
       return rectsInsideBound;
   }

}