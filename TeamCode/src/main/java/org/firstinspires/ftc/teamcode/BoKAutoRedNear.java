package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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

        opMode.sleep(1000);
        robot.jewelFlicker.setPosition(robot.JF_INIT);
        robot.jewelArm.setPosition(robot.JA_MID);
        //Widen glyph claw
        robot.clawGrab.setPosition(robot.CG_MID);
        //lower glyph claw
        for(double i = robot.clawWrist.getPosition() ; i>0.89 ; i-=0.01 )
        {
            robot.clawWrist.setPosition(i);
        }
        //raise the flicker again
        robot.jewelArm.setPosition(robot.JA_INIT);

        robot.resetDTEncoders();
        robot.startMove(0.2,0.2,30,true);
        while (opMode.opModeIsActive() &&
                (robot.areDTMotorsBusy())) {
        }

        // Stop all motion;
        robot.stopMove();

    }
}
