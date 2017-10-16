package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoRedNear extends BoKAutoCommon
{
    @Override
    public void runSoftware()
    {
        // detect Vuforia image
        getCryptoColumn();
        // setup flicker
        setJewelFlicker();

        opMode.sleep(500);
        if (foundRedOnLeft)
            robot.jewelFlicker.setPosition(1);
        else
            robot.jewelFlicker.setPosition(0);

        opMode.sleep(500);
        robot.jewelFlicker.setPosition(robot.JF_INIT);
        robot.jewelArm.setPosition(robot.JA_MID);
    }
}
