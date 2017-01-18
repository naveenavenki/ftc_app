package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Krishna Saxena on 10/14/2016.
 * Define all the CONSTANTS used for autonomous mode.
 */
public interface BoKAuto {

    public enum BoKAlliance {
        BOK_ALLIANCE_RED,
        BOK_ALLIANCE_BLUE
    }


    static final float WHITE_LINE = 0.4f;
    static final float LINE_EDGE = 0.25f;
    static final double LEFT_POWER_LINE_FOLLOW = 0.2;
    static final double RIGHT_POWER_LINE_FOLLOW = 0.225;

    static final float LEFT_MOTOR_POWER = 0.5f;
    static final float RIGHT_MOTOR_POWER = 0.5f;

    static final double WAIT_FOR_SEC_SHOOTER = 5.0;
    static final int ROBOT_DISTANCE_TO_PUSH_THE_BEACON = 9;
    static final int ROBOT_DISTANCE_FROM_WALL_INITIAL = 15;
    static final int ROBOT_DISTANCE_FROM_WALL_FOR_BEACON = 16;
    static final int ROBOT_DISTANCE_FROM_WALL_AFTER_BEACON = 32;

    static final double ROBOT_BATTERY_LEVEL_HIGH_THRESHOLD = 13.0;
    static final double ROBOT_BATTERY_LEVEL_MED_THRESHOLD = 12.4;
    static final double SHOOTER_MOTOR_POWER_CHANGE = 0.1;

    static final double POWER_REDUCTION_FACTOR_FWD = 1.5;
    static final double POWER_REDUCTION_FACTOR_TURN = 2.5;

    static final double MOVE_FORWARD_TO_PARK  = 45.0; // in inches

    static final double ONE_SECOND = 1.0;
    static final double TWO_SECONDS = 2.0;
    static final double THREE_SECONDS = 3.0;
    static final double FOUR_SECONDS = 4.0;

    static final long SLEEP_100_MS = 100;
    static final long SLEEP_250_MS = 250;

    public void initSoftware(LinearOpMode opMode, BoKHardwareBot robot, BoKAlliance redOrBlue);
    public void runSoftware(LinearOpMode opMode, BoKHardwareBot robot) throws InterruptedException;
}
