package org.firstinspires.ftc.teamcode.auto.utility;

import android.os.Environment;
import java.io.File;

/**
 * Instantiating Configuration will read a file and return a list of commands and properties.
 *
 * @author Trinity Chung
 * @version 0.0
 */
public class Configuration {

    private final File FOLDER = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS);

    public Configuration(String fileName) {

        try {
            File file = new File(FOLDER, fileName);

            //TODO: Read the file and get a list of commands/properties!

        } catch (Exception ex) {
            throw new RuntimeException("Configuration", ex);
        }

    }
}
