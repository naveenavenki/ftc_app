package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoBlueFar extends BoKAutoCommon
{
    @Override
    public void runSoftware() {
        // move backwards
        // detect Vuforia image
        getCryptoColumn(VUFORIA_TIMEOUT);
        // setup flicker
        setJewelFlicker();

        opMode.sleep(500);
        if (foundRedOnLeft)
            robot.jewelFlicker.setPosition(0);
        else
            robot.jewelFlicker.setPosition(1);

        opMode.sleep(1000);
        robot.jewelFlicker.setPosition(robot.JF_INIT);
        robot.jewelArm.setPosition(robot.JA_MID);
        robot.jewelFlicker.setPosition(robot.JF_INIT);
        robot.jewelArm.setPosition(robot.JA_MID);
        //Widen glyph claw
        robot.glyphArm.clawGrab.setPosition(robot.CG_MID);
        //lower glyph claw
        for(double i = robot.glyphArm.clawWrist.getPosition() ; i>0.89 ; i-=0.01 )
        {
            robot.glyphArm.clawWrist.setPosition(i);
        }
        //raise the flicker again
        robot.jewelArm.setPosition(robot.JA_INIT);

        robot.resetDTEncoders();
        robot.startMove(0.2,0.2,25,false);
        while (opMode.opModeIsActive() &&
                (robot.areDTMotorsBusy())) {
        }

        // Stop all motion;
        robot.stopMove();
    }
}
