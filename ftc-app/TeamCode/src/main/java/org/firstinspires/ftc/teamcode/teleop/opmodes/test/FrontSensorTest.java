package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.teamcode.hardware.sensor.MaxSonarAnalogSensor;
import org.firstinspires.ftc.teamcode.hardware.sensor.MaxSonarI2CXL;
import org.firstinspires.ftc.teamcode.hardware.sensor.SharpDistanceSensor;
import org.firstinspires.ftc.teamcode.teleop.opmodes.drive.BaseDrive;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;

@TeleOp(group="Test")
public class FrontSensorTest extends BaseDrive {

    MaxSonarAnalogSensor frontSensor;

    Button button = new Button();

    @Override
    protected void initialize() {
        super.initialize();
        robot.driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontSensor = new MaxSonarAnalogSensor(robot.getHardwareMap(), "front sensor");
    }

    @Override
    protected void update() {
        super.update();
        telemetry.addData("Distance", frontSensor.getDistance());
        telemetry.addData("Voltage", frontSensor.getVoltage());
        telemetry.update();
    }

}
