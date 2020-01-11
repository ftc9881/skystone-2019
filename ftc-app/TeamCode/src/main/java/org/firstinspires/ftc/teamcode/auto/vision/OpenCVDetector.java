package org.firstinspires.ftc.teamcode.auto.vision;

import org.opencv.core.Mat;
import org.opencv.core.Size;

import org.openftc.easyopencv.OpenCvPipeline;

/**
 * Modified DogeCVDetector.
 */
public abstract class OpenCVDetector extends OpenCvPipeline {

    public enum Stage {
        FINAL_DISPLAY,
        THRESHOLD,
        CONTOURS,
        RAW_IMAGE
    }

    public abstract Mat process(Mat input);
    public abstract void useDefaults();

    protected boolean found = false;

    private Mat workingMat = new Mat();

    protected Stage stageToRenderToViewport = Stage.FINAL_DISPLAY;
    private Stage[] stages = Stage.values();

    public boolean isDetected(){
        return found;
    }

    @Override
    public final Mat processFrame(Mat input) {
        input.copyTo(workingMat);
        if (workingMat.empty()) {
            return input;
        }
        workingMat = process(workingMat);
        return workingMat;
    }

    @Override
    public void onViewportTapped()
    {
        int currentStageNum = stageToRenderToViewport.ordinal();
        int nextStageNum = currentStageNum + 1;
        if (nextStageNum >= stages.length) {
            nextStageNum = 0;
        }
        stageToRenderToViewport = stages[nextStageNum];
    }

}
