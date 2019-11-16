package org.firstinspires.ftc.teamcode.auto.structure;

import android.os.Environment;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.json.JSONArray;
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
 */
public class Configuration {

    private final String PATH = Environment.getExternalStorageDirectory().getAbsolutePath() + "/Robot/";
    public ArrayList<Command> commands;
    public Command properties;

    public Configuration(String fileName) {

        try {
            String fileContents = readFile(fileName);
            JSONObject config = new JSONObject(fileContents);
            commands = new ArrayList();

            JSONArray allCommandsJson = config.getJSONArray("commands");

            for (int i = 0; i < allCommandsJson.length(); i++) {
                commands.add(new Command(allCommandsJson.getJSONObject(i)));
            }

            // create properties
            properties = new Command("PROPERTIES", config.getJSONObject("properties"));

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
