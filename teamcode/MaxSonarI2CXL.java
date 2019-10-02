package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

import static java.lang.Thread.sleep;

/**
 * Created by dkrider on 10/27/2017.
 */

@I2cSensor(name = "MaxSonar I2CXL", description = "MaxSonar I2CXL Sensor from MaxBotix", xmlTag = "MaxSonarI2CXL")
public class MaxSonarI2CXL extends I2cDeviceSynchDevice<I2cDeviceSynch> implements SonarIF, Runnable {
    private Thread _thread;

    private boolean _autoPing;

    private int _autoPingDelay;

    private long _lastPingTime = Long.MAX_VALUE;

    private int _lastDistance = Integer.MAX_VALUE;

    private long _minDelay = 100;

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    @Override
    public String getDeviceName()
    {

        return "MaxSonar I2CXL";
    }

    public MaxSonarI2CXL(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(I2cAddr.create8bit(0xE0));

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    public void setI2cAddress(I2cAddr i2cAddr) {
        deviceClient.setI2cAddress(i2cAddr);
    }

    public void ping() {
        _lastPingTime = System.currentTimeMillis();
        deviceClient.write8(0, 0x51, I2cWaitControl.ATOMIC);
    }

    public void setMinDelay(int minDelay) {
        _minDelay = minDelay;
    }

    public void startAutoPing(int delay) {
        if(_thread == null) {
            _thread = new Thread(this);
        }

        _autoPingDelay = delay;
        _autoPing = true;

        _thread.start();
    }

    public void stopAutoPing() {
        _autoPing = false;
        _thread = null;
    }

    public double getDistance() {
        long currentTimeMillis = System.currentTimeMillis();
        int distance;

        if(currentTimeMillis - _lastPingTime < _minDelay) {
            distance = _lastDistance;
        }
        else {
            distance = TypeConversion.byteArrayToShort(deviceClient.read(0x01, 2));
            _lastDistance = distance;
        }

        return distance;
    }

    @Override
    public void run() {
        while(_autoPing) {
            ping();
            try {
                Thread.sleep(_autoPingDelay);
            } catch (InterruptedException e) {
            }
        }
    }
}
