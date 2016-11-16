package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Krishna Saxena on 10/14/2016.
 */
public interface BoKAuto {

    public enum BoKAlliance {
        BOK_ALLIANCE_RED,
        BOK_ALLIANCE_BLUE
    }


    static final float LEFT_MOTOR_POWER = 0.5f;
    static final float RIGHT_MOTOR_POWER = 0.525f;

    public void initSoftware(LinearOpMode opMode, BoKHardwareBot robot, BoKAlliance redOrBlue);
    public void runSoftware(LinearOpMode opMode, BoKHardwareBot robot) throws InterruptedException;
}
