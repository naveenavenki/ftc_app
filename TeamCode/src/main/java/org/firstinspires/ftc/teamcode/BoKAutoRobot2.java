package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoRobot2 extends BoKAutoCommon
{
    // Constructor
    public BoKAutoRobot2()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_BLUE;
    }

    @Override
    public void runSoftware()
    {
        robot.jewelFlicker.setPosition(robot.JF_FINAL);
        opMode.sleep(WAIT_FOR_SERVO_MS);

        // NOTE: Move backwards towards crypto
        robot.jewelArm.setPosition(robot.JA_MID);
        opMode.sleep(WAIT_FOR_SERVO_MS * 3); // let the flicker settle down

        double cmCurrent = robot.rangeSensorJA.cmUltrasonic();
        while (opMode.opModeIsActive() && (cmCurrent > 20))
        {
            cmCurrent = robot.rangeSensorJA.cmUltrasonic();
        }
        double distanceToMove = (cmCurrent/2.54) + 3.94;

        // Raise the jewel arm
        robot.jewelArm.setPosition(robot.JA_INIT);
        opMode.sleep(WAIT_FOR_SERVO_MS); // temp

        Log.v("BOK", "cmCurrent: " + cmCurrent + ", dist: " + distanceToMove );
        move(DT_POWER_FOR_CRYPTO, DT_POWER_FOR_CRYPTO, distanceToMove, false, 5);
        //move(0.2, 0.2, 8, false, 5);

        //opMode.sleep(5000);

        // Move back out of the balancing stone
        //moveRamp(0.5, 48, true, 5);
        //moveRamp(0.5, 48, false, 5);
        //strafeRamp(0.35, 2.5, true, 4);
        //strafeRamp(0.35, 2.5, false, 4);

    }
}
