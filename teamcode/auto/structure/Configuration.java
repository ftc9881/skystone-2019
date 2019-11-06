package org.firstinspires.ftc.teamcode.auto.structure;

import android.os.Environment;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;

/**
 * Instantiating Configuration will read a file and return a list of commands and properties.
 *
 * @author Trinity Chung
 * @version 0.0
 */
public class Configuration {

    private final String PATH = Environment.getExternalStorageDirectory().getAbsolutePath() + "/Robot/";
    public ArrayList<Command> commands;
    public JSONObject properties;

    public Configuration(String fileName) {

        try {
            String fileContents = readFile(fileName);
            JSONObject config = new JSONObject(fileContents);
            commands = new ArrayList();

            JSONObject allCommandsJson = config.getJSONObject("commands");
            Iterator<String> allCommandsKeys = config.getJSONObject("commands").keys();

            while (allCommandsKeys.hasNext()) {
                String name = allCommandsKeys.next();
                JSONObject commandJson = allCommandsJson.getJSONObject(name);
                commands.add(new Command(name, commandJson));
            }

            // create properties
            properties = config.getJSONObject("properties");

        } catch (Exception ex) {
            throw new RuntimeException("Configuration", ex);
        }
    }

    private String readFile(String name) throws IOException {
        File file = new File(PATH, name);
        BufferedReader br = new BufferedReader(new FileReader(file));
        StringBuilder sb = new StringBuilder();
        String line;
        do {
            line = br.readLine();
            sb.append(line);
        } while (line != null);
        return sb.toString();
    }
}
