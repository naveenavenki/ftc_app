package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Krishna Saxena on 10/14/2016.
 * Define all the CONSTANTS used for autonomous mode.
 */
public interface BoKAuto
{
    public enum BoKAllianceColor {
        BOK_ALLIANCE_RED,
        BOK_ALLIANCE_BLUE
    }

    public enum BoKAutoStatus {
        BOK_AUTO_FAILURE,
        BOK_AUTO_SUCCESS
    }

    public static long WAIT_FOR_SERVO_MS = 500;
    public static double VUFORIA_TIMEOUT = 4.0;
    public static double TOUCH_CRYPTO_TIMEOUT = 5.0;
    public static double DT_BACK_TIMEOUT = 2.0;
    public static double DT_STRAFE_TIMEOUT = 3.0;

    public static double DT_POWER_FOR_STONE = 0.2;
    public static double DT_POWER_FOR_STRAFE = 0.2;
    public static double DT_POWER_FOR_CRYPTO = 0.15;
    public static double DT_POWER_FOR_BACK = 0.2;
    public static double UPPER_ARM_POWER = 0.3;

    public static double DISTANCE_BACK_FOR_CRYPTO = 3.75;
    public static double DEGREES_UPPER_ARM_FOR_GLYPH = 40;

    public BoKAutoStatus initSoftware(LinearOpMode opMode,
                                      BoKHardwareBot robot,
                                      BoKAllianceColor redOrBlue);

    public void runSoftware();
}
