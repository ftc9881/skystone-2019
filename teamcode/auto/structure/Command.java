package org.firstinspires.ftc.teamcode.auto.structure;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.json.JSONException;
import org.json.JSONObject;

public class Command {

    private static final String TAG = "Command";
    public String name;
    private JSONObject json;

    public Command(String name, JSONObject json) {
        this.name = name;
        this.json = json;
    }

    public Command(JSONObject json) {
        try {
            this.name = json.getString("command");
        } catch (JSONException ex) {
            this.name = "???";
        }
        this.json = json;
    }

    public String getString(String key, String defaultValue) {
        String value = defaultValue;
        try {
            value = json.getString(key);
        } catch (JSONException ex) {
            AutoRunner.log(TAG, "Missing JSON key: " + key);
        }
        return value;
    }

    public double getDouble(String key, double defaultValue) {
        double value = defaultValue;
        try {
            value = json.getDouble(key);
        } catch (JSONException ex) {
            AutoRunner.log(TAG, "Missing JSON key: " + key);
        }
        return value;
    }

    public int getInt(String key, int defaultValue) {
        int value = defaultValue;
        try {
            value = json.getInt(key);
        } catch (JSONException ex) {
            AutoRunner.log(TAG, "Missing JSON key: " + key);
        }
        return value;
    }
    
    public boolean getBoolean(String key, boolean defaultValue) {
        boolean value = defaultValue;
        try {
            String rawValue = json.getString(key).toLowerCase();
            value = rawValue.contains("true") || rawValue.contains("y") || rawValue.contains("1");
        } catch (JSONException ex) {
            AutoRunner.log(TAG, "Missing JSON key: " + key);
        }
        return value;
    }

    public AngleUnit getAngleUnit(String key, AngleUnit defaultValue) {
        AngleUnit value = defaultValue;
        try {
            String rawValue = json.getString(key).toLowerCase();
            if (rawValue.contains("rad")) {
                value = AngleUnit.RADIANS;
            } else if (rawValue.contains("deg")) {
                value = AngleUnit.DEGREES;
            }
        } catch (JSONException ex) {
            AutoRunner.log(TAG, "Missing JSON key: " + key);
        }
        return value;
    }

    public Angle getAngle(String key, double defaultValue, AngleUnit unit) {
        double value = getDouble(key, defaultValue);
        return new Angle(value, unit);
    }


}
