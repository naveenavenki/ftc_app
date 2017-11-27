package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Locale;

/**
 * Created by Krishna on 11/25/2017.
 */

public class BoKLogInfo
{
    private static FileWriter writer = null;
    private static String logStr;

    protected static void openLog()
    {
        String logFileName = AppUtil.getInstance().ROBOT_SETTINGS + "/BoKLog.txt";
        try {
            writer = new FileWriter(logFileName, true); // append mode
        } catch (IOException e) {
            Log.v("BOK", "Could not open log file" + e.getMessage());
        }
    }

    protected static void logInfo(String log)
    {
        if (writer != null) {
            //try {
                SimpleDateFormat serverFormat =
                        new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.getDefault());
                String currentTime = serverFormat.format(Calendar.getInstance().getTime());
                //Log.v("BOK", currentTime + ": writing " + log);
                //writer.write(currentTime + ": " + log);
                logStr += currentTime + ": " + log + "\n";
            //} catch (IOException e) {
            //    Log.v("BOK", "Could not write to log file" + e.getMessage());
            //}
        }
    }

    protected static void closeLog()
    {
        if (writer != null) {
            try {
                writer.write(logStr);
                writer.close();
            } catch (IOException e) {
                Log.v("BOK", "Could not write & close log file" + e.getMessage());
            }
        }
    }
}
