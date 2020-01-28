package org.firstinspires.ftc.teamcode.robot.devices;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxFirmwareVersionManager;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

public class OptimizedIMU {

    OptimizedI2cDevice imu;

    public OptimizedIMU(LynxModule module, int bus) {
        this.imu = new OptimizedI2cDevice( LynxFirmwareVersionManager.createLynxI2cDeviceSynch(AppUtil.getDefContext(), module, bus), true);
    }

    public double getHeading() {
        return 0;
    }

}
