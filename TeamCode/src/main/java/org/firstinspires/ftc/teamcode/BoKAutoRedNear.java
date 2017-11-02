package org.firstinspires.ftc.teamcode;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoRedNear extends BoKAutoCommon {
    private static double TIMEOUT_RIGHT = 4;
    private static double TIMEOUT_CENTER = 5;
    private static double TIMEOUT_LEFT = 6;

    @Override
    public void runSoftware() {
        // Detect Vuforia image
        getCryptoColumn(VUFORIA_TIMEOUT);
        // Setup flicker
        setJewelFlicker();
        opMode.sleep(WAIT_FOR_SERVO_MS);

        if (foundRedOnLeft) {
            robot.jewelFlicker.setPosition(robot.JF_RIGHT);
        } else {
            robot.jewelFlicker.setPosition(robot.JF_LEFT);
        }
        opMode.sleep(WAIT_FOR_SERVO_MS);

        // Raise the flicker arm and position the flicker to face the cryptobox
        robot.jewelFlicker.setPosition(robot.JF_FINAL);
        robot.jewelArm.setPosition(robot.JA_INIT);

        // Move forward out of balancing stone
        // Distance and timeout depends on column number; TBD
        move(DT_POWER_FOR_STONE, DT_POWER_FOR_STONE, 15, true, TIMEOUT_RIGHT);

        // Prepare the jewel arm & the optical color/range sensor
        robot.jewelArm.setPosition(robot.JA_MID);
        opMode.sleep(WAIT_FOR_SERVO_MS);

        // Strafe to the right
        strafe(DT_POWER_FOR_STRAFE, ROTATIONS_STRAFE_TO_WALL, true, DT_STRAFE_TIMEOUT);

        // Move forward towards cryptobox using optical color/range sensor
        moveTowardsCrypto(DT_POWER_FOR_CRYPTO, DISTANCE_TO_CRYPTO, true, CRS_CRYPTO_TIMEOUT);
/*
        move(DT_POWER_FOR_BACK,
             DT_POWER_FOR_BACK,
             DISTANCE_BACK_FOR_CRYPTO,
             false,
             DT_BACK_TIMEOUT);

        // Determine how many rotations to strafe to the left?
        strafe(DT_POWER_FOR_STRAFE, 0.4, false, DT_STRAFE_TIMEOUT);

        // Now prepare to unload the glyph
        robot.glyphArm.moveUpperArm(DEGREES_UPPER_ARM_FOR_GLYPH, UPPER_ARM_POWER);

        for (double i = robot.glyphArm.clawWrist.getPosition(); i > robot.CW_MID; i -= 0.01) {
            robot.glyphArm.clawWrist.setPosition(i);
            opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
        }

        robot.glyphArm.clawGrab.setPosition(robot.CG_INIT);

        robot.glyphArm.moveUpperArm(-DEGREES_UPPER_ARM_FOR_GLYPH, UPPER_ARM_POWER);
        */
    }
}
