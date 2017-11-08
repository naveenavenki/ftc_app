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
    public static double CRS_CRYPTO_TIMEOUT = 5.0;
    public static double DT_STRAFE_TIMEOUT = 3.0;
    public static double DT_TURN_TIMEOUT = 3.0;
    public static double BLUE_CRYPTO_BACK_TIMEOUT = 1.0;

    public static double DT_POWER_FOR_STONE = 0.2;
    public static double DT_POWER_FOR_STRAFE = 0.15;
    public static double DT_POWER_FOR_RS = 0.15;
    public static double DT_POWER_FOR_CRYPTO = 0.1;
    public static double DT_TURN_SPEED_LOW  = 0.15;
    public static double DT_TURN_SPEED_HIGH = 0.4;

    //public static double UPPER_ARM_POWER = 0.2;

    public static double DISTANCE_TO_CRYPTO_CM = 4.0;
    public static double DISTANCE_BLUE_BACK_TO_COLUMN = 3.85;
    //public static double DISTANCE_TO_COLUMN = 1.5;
    public static double ROTATIONS_STRAFE_TO_WALL = 0.15;
    //public static double ROTATIONS_STRAFE_FROM_WALL = 0.25;
    public static double DEGREES_UPPER_ARM_FOR_GLYPH = 20;

    public BoKAutoStatus initSoftware(LinearOpMode opMode,
                                      BoKHardwareBot robot,
                                      BoKAllianceColor redOrBlue);

    public void runSoftware();
}
