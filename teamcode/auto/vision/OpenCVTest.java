package org.firstinspires.ftc.teamcode.auto.vision;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.disnodeteam.dogecv.detectors.skystone.StoneDetector;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;
import com.disnodeteam.dogecv.scoring.PerfectAreaScorer;
import com.disnodeteam.dogecv.scoring.RatioScorer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.TeleOpBase;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp
public class OpenCVTest extends TeleOpBase {
    private CustomSkystoneDetector skystoneDetector;
    OpenCvCamera phoneCam;

    @Override
    protected void initialize() {

        Rect lookWindow = new Rect();
        lookWindow.x = config.getInt("look window x", 0);
        lookWindow.y = config.getInt("look window y", 0);
        lookWindow.width = config.getInt("look window w", 320);
        lookWindow.height = config.getInt("look window h", 240);
        int targetArea = config.getInt("area", 5000);
        double targetRatio = config.getDouble("ratio", .0625);
        double areaWeight = config.getDouble("area weight", 1);
        double ratioWeight = config.getDouble("ratio weight", 3);

        // Default scorers from DogeCV
        RatioScorer ratioScorer = new RatioScorer(targetRatio, ratioWeight); // Used to find the short face of the stone
        PerfectAreaScorer perfectAreaScorer = new PerfectAreaScorer(targetArea,areaWeight); // Used to find objects near a tuned area value

        skystoneDetector = new CustomSkystoneDetector();
        skystoneDetector.setLookWindow(lookWindow);
        skystoneDetector.addScorer(ratioScorer);
        skystoneDetector.addScorer(perfectAreaScorer);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();
        phoneCam.setPipeline(skystoneDetector);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    @Override
    protected void update() {
        telemetry.addData("Rect Area", skystoneDetector.foundRectangle().area());
        telemetry.addData("Rect X", skystoneDetector.foundRectangle().x);
        telemetry.addData("Rect Y", skystoneDetector.foundRectangle().y);
        telemetry.addData("Rect W", skystoneDetector.foundRectangle().width);
        telemetry.addData("Rect H", skystoneDetector.foundRectangle().height);
        telemetry.update();
    }

}
