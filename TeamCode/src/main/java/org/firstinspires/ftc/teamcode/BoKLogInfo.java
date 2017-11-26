package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Locale;

/**
 * Created by shiv on 11/25/2017.
 */

public class BoKLogInfo
{
    private static boolean isOpen = false;
    private static FileWriter writer = null;

    protected static void logInfo(String log)
    {
        if (!isOpen) {
            Log.v("BOK", "Not open");
            String logFileName = AppUtil.getInstance().ROBOT_SETTINGS + "/BoKLog.txt";
            try {
                writer = new FileWriter(logFileName, true);
                isOpen = true;
                Log.v("BOK", "Open " + logFileName);
            } catch (IOException e) {
                Log.v("BOK", "Could not open log file" + e.getMessage());
            }
        }
        if (isOpen) {
            try {
                SimpleDateFormat serverFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.getDefault());
                String currentTime = serverFormat.format(Calendar.getInstance().getTime());
                Log.v("BOK", "writing " + log + "date " + currentTime);
                writer.write(currentTime + ": " + log);
            } catch (IOException e) {
                Log.v("BOK", "Could not write to log file" + e.getMessage());
            }
        }
    }

    protected static void closeLog()
    {
        if (writer != null) {
            try {
                writer.close();
            } catch (IOException e) {
                Log.v("BOK", "Could not close log file" + e.getMessage());
            }
        }
    }
}
