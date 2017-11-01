package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoRedNear extends BoKAutoCommon {
    private static double TIMEOUT_RIGHT = 4;
    private static double TIMEOUT_CENTER = 5;
    private static double TIMEOUT_LEFT = 6;

    @Override
    public void runSoftware() {
        moveWithRangeSensor(false, 20, 4);
/*
        // detect Vuforia image
        getCryptoColumn(VUFORIA_TIMEOUT);
        // setup flicker
        setJewelFlicker();

        opMode.sleep(WAIT_FOR_SERVO_MS);
        if (foundRedOnLeft) {
            robot.jewelFlicker.setPosition(1);
            opMode.telemetry.addData("BOK", "Red is found on left");
        } else {
            robot.jewelFlicker.setPosition(0);
            opMode.telemetry.addData("BOK", "Red is found on right");
        }
        opMode.telemetry.update();
        opMode.sleep(WAIT_FOR_SERVO_MS);

        //raise the flicker again
        robot.jewelFlicker.setPosition(robot.JF_HIT_CRYPTO);
        robot.jewelArm.setPosition(robot.JA_INIT);

        // Distance and timeout depends on column number; TBD
        move(DT_POWER_FOR_STONE, DT_POWER_FOR_STONE, 15, true, TIMEOUT_RIGHT);

        robot.jewelArm.setPosition(robot.JA_MID);
        opMode.sleep(WAIT_FOR_SERVO_MS);

        // Determine how many rotations to strafe to the right?
        strafe(DT_POWER_FOR_STRAFE, 1, true, DT_STRAFE_TIMEOUT);

        // move forward towards cryptobox
        // Know if we ran out of time?
        if (hitCryptoWithTouch(true, TOUCH_CRYPTO_TIMEOUT)) {

            move(DT_POWER_FOR_BACK,
                 DT_POWER_FOR_BACK,
                 DISTANCE_BACK_FOR_CRYPTO,
                 false,
                 DT_BACK_TIMEOUT);

            // Determine how many rotations to strafe to the left?
            strafe(DT_POWER_FOR_STRAFE, 1, false, DT_STRAFE_TIMEOUT);

            // Now prepare to unload the glyph
            robot.glyphArm.moveUpperArm(DEGREES_UPPER_ARM_FOR_GLYPH, UPPER_ARM_POWER);

            for (double i = robot.glyphArm.clawWrist.getPosition(); i > robot.CW_MID; i -= 0.01) {
                robot.glyphArm.clawWrist.setPosition(i);
                opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
            }

            robot.glyphArm.clawGrab.setPosition(robot.CG_INIT);

            robot.glyphArm.moveUpperArm(-DEGREES_UPPER_ARM_FOR_GLYPH, UPPER_ARM_POWER);
        }
        */
    }
}
