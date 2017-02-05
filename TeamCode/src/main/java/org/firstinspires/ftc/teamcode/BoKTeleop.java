package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Krishna on 11/26/2016.
 * Define all the CONSTANTS used for teleop mode.
 */

public interface BoKTeleop {
    static final int METRONOME_TICK = 40; // 40 milli-seconds
    static final double CAPBALL_LIFT_POWER         = 1.0;
    static final double GAME_STICK_DEAD_ZONE      = 0.05;
    static final double DT_POWER_LOW_THRESHOLD    = 0.3;
    static final double DT_POWER_HIGH_THRESHOLD   = 1;
    static final double DT_POWER_REDUCTION_FACTOR = 0.25;
    static final double CAPBALL_LIFT_DEAD_ZONE    = 0.2;
    static final double SHOOTER_ANGLE_REDUCTION   = 0.07;

    public void initSoftware(LinearOpMode opMode, BoKHardwareBot robot, BoKAuto.BoKAlliance redOrBlue);
    public void runSoftware(LinearOpMode opMode, BoKHardwareBot robot);
    public void moveRobot(LinearOpMode opMode, BoKHardwareBot robot);
}
