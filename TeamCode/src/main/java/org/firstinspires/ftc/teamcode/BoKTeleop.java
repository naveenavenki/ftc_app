package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by shiv on 11/26/2016.
 */

public interface BoKTeleop {
    static final int METRONOME_TICK = 40; // 40 milli-seconds
    static final double CAPBALL_LIFT_POWER = 1.0;

    public void initSoftware(LinearOpMode opMode, BoKHardwareBot robot, BoKAuto.BoKAlliance redOrBlue);
    public void runSoftware(LinearOpMode opMode, BoKHardwareBot robot) throws InterruptedException;
}
