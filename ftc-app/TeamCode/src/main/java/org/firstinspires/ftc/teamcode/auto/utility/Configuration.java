package org.firstinspires.ftc.teamcode.auto.utility;

import android.os.Environment;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

/**
 * Instantiating Configuration will read a file and return a list of commands and properties.
 *
 * @author Trinity Chung
 * @version 0.0
 */
public class Configuration {

    private final String PATH = Environment.getExternalStorageDirectory().getAbsolutePath() + "/Robot/";
    public JSONArray commands;
    public JSONObject properties;

    public Configuration(String fileName) {

        try {

            File file = new File(PATH, fileName);
            BufferedReader br = new BufferedReader(new FileReader(file));
            StringBuilder sb = new StringBuilder();
            String line;
            do {
                line = br.readLine();
                sb.append(line);
            } while (line != null);
            String fileContents = sb.toString();

            JSONObject config = new JSONObject(fileContents);
            commands = config.getJSONArray("commands");
            properties = config.getJSONObject("properties");

        } catch (Exception ex) {
            throw new RuntimeException("Configuration", ex);
        }

    }
}
