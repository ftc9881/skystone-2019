package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous
public class TestVuforiaAutoOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();

        Vuforia vuforia = new Vuforia(hardwareMap);
        vuforia.initialize();
        vuforia.startLook(VisionSystem.TargetType.SKYSTONE);

        while (opModeIsActive()) {
            if (vuforia.found()) {
                telemetry.addData("Pose", vuforia.getPose().toString());
            }
            telemetry.update();
        }
    }

}
