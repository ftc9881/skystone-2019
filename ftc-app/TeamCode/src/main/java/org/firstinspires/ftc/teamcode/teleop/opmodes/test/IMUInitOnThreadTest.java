package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.SomethingBadHappened;
import org.firstinspires.ftc.teamcode.teleop.opmodes.drive.BaseDrive;

@TeleOp(group="Test")
public class IMUInitOnThreadTest extends BaseDrive {

    InitIMUOnThread initIMUAction;

    @Override
    protected void initialize() {
        super.initialize();
        initIMUAction = new InitIMUOnThread();
        initIMUAction.start();
    }

    @Override
    protected void update() {
        super.update();
        if (robot.imuIsInitialized()) {
            telemetry.addData("imu", robot.imu.getHeading().getDegrees());
            telemetry.addData("imu integrated", robot.imu.getIntegratedHeading().getDegrees());
        }
        else {
            telemetry.addData("initializing", "imu...");
        }
        telemetry.update();
    }

    class InitIMUOnThread extends Action {
        @Override
        protected void onRun() {
            robot.initializeIMU();
        }

        @Override
        protected boolean runIsComplete() {
            return robot.imuIsInitialized();
        }

        @Override
        protected void insideRun() throws SomethingBadHappened { }

        @Override
        protected void onEndRun() throws SomethingBadHappened { }
    }

}
