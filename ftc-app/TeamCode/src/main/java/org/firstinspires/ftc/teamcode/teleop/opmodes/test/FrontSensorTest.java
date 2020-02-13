package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.teamcode.hardware.sensor.MaxSonarI2CXL;
import org.firstinspires.ftc.teamcode.hardware.sensor.SharpDistanceSensor;
import org.firstinspires.ftc.teamcode.teleop.opmodes.drive.BaseDrive;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;

@TeleOp(group="Test")
public class FrontSensorTest extends BaseDrive {

    MaxSonarI2CXL frontSensor;

    Button button = new Button();

    @Override
    protected void initialize() {
        super.initialize();
        frontSensor = hardwareMap.get(MaxSonarI2CXL.class, "front sensor");

//        frontSensor.setI2cAddress(I2cAddr.create8bit(0xE0));
        frontSensor.setI2cAddress(I2cAddr.create7bit(0x00));
//        frontSensor.setI2cAddress(I2cAddr.create8bit(0xDE));
    }

    @Override
    protected void update() {

        if (button.is(Button.State.DOWN)) {
            I2cAddr addr = frontSensor.getDeviceClient().getI2cAddress();
            frontSensor.setI2cAddress(I2cAddr.create7bit(0x00));
        }


        super.update();
        telemetry.addData("Distance", frontSensor.getDistance());
        telemetry.addData("Connection", frontSensor.getConnectionInfo());
        telemetry.update();
    }

}
