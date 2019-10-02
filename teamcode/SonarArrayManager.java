package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.TypeConversion;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Vector;

/**
 * Created by dkrider on 10/27/2017.
 */

public class SonarArrayManager implements Runnable {
    private Map<String, SonarIF> _sonars = new LinkedHashMap<String, SonarIF>();

    private boolean _autoPing;

    private int _autoPingDelay;

    private Thread _thread;

    public void addSonar(String sonarID, SonarIF sonar) {
        _sonars.put(sonarID, sonar);
    }

    public double getDistance(String sonarID) {
        return _sonars.get(sonarID).getDistance();
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

    @Override
    public void run() {
        while(_autoPing) {
            for(Map.Entry<String, SonarIF> sonarEntry : _sonars.entrySet()) {
                sonarEntry.getValue().ping();

                try {
                    Thread.sleep(_autoPingDelay);
                } catch (InterruptedException e) {
                }
            }
        }
    }
}
