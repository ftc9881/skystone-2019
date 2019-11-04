package org.firstinspires.ftc.teamcode.auto.structure;


import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.auto.actions.RelativeMove;
import org.json.JSONException;
import org.json.JSONObject;

public class Command {

    public final String name;
    private JSONObject json;

    public Command(String name, JSONObject json) {
        this.name = name;
        this.json = json;
    }

    public String getString(String key, String defaultValue) {
        String value = defaultValue;
        try {
            value = json.getString(key);
        } catch (JSONException ex) {
            RobotLog.e("Missing JSON key: " + key);
        }
        return value;
    }

    public double getDouble(String key, double defaultValue) {
        double value = defaultValue;
        try {
            value = json.getDouble(key);
        } catch (JSONException ex) {
            RobotLog.e("Missing JSON key: " + key);
        }
        return value;
    }

    public int getInt(String key, int defaultValue) {
        int value = defaultValue;
        try {
            value = json.getInt(key);
        } catch (JSONException ex) {
            RobotLog.e("Missing JSON key: " + key);
        }
        return value;
    }
    
    public boolean getBoolean(String key, boolean defaultValue) {
        boolean value = defaultValue;
        try {
            String rawValue = json.getString(key).toLowerCase();
            value = rawValue.contains("true") || rawValue.contains("y") || rawValue.contains("1");
        } catch (JSONException ex) {
            RobotLog.e("Missing JSON key: " + key);
        }
        return value;
    }

    public RelativeMove.Direction getDirection(String key, RelativeMove.Direction defaultValue) {
        RelativeMove.Direction value = defaultValue;
        try {
            String rawValue = json.getString(key).toLowerCase();
            switch (rawValue) {
                case "front":
                case"f":
                    value = RelativeMove.Direction.FRONT;
                    break;
                case "left":
                case"l":
                    value = RelativeMove.Direction.LEFT;
                    break;
                case "right":
                case"r":
                    value = RelativeMove.Direction.RIGHT;
                    break;
                case "back":
                case"b":
                    value = RelativeMove.Direction.BACK;
                    break;
            }
        } catch (JSONException ex) {
            RobotLog.e("Missing JSON key: " + key);
        }
        return value;
    }

}
