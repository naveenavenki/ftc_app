package org.firstinspires.ftc.teamcode;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoBlueFar extends BoKAutoCommon
{
    private static double DT_TIMEOUT = 4;
    private static double TIMEOUT_LEFT = 4;
    private static double TIMEOUT_CENTER = 5;
    private static double TIMEOUT_RIGHT = 6;
    private static int TURN_RIGHT_DEGREES = -90;

    @Override
    public void runSoftware() {
        // NOTE: Move backwards towards crypto
        // Detect Vuforia image
        getCryptoColumn(VUFORIA_TIMEOUT);
        // Setup flicker
        setJewelFlicker();

        opMode.sleep(WAIT_FOR_SERVO_MS);
        if (foundRedOnLeft)
            robot.jewelFlicker.setPosition(robot.JF_LEFT);
        else
            robot.jewelFlicker.setPosition(robot.JF_RIGHT);
        opMode.sleep(WAIT_FOR_SERVO_MS);

        // Raise the flicker arm and position the flicker to face the cryptobox
        robot.jewelFlicker.setPosition(robot.JF_FINAL);
        robot.jewelArm.setPosition(robot.JA_INIT);

        // Move out of the balancing stone, distance: TBD?
        move(DT_POWER_FOR_STONE, DT_POWER_FOR_STONE, 15, false, DT_TIMEOUT);

        // turn right 90 degrees
        gyroTurn(DT_TURN_SPEED_HIGH, TURN_RIGHT_DEGREES, DT_TURN_TIMEOUT);

        // Move towards wall: TBD? (if there isn't enough space)

        // Move back towards cryptobox
        // Distance and timeout depends on column number; TBD
        move(DT_POWER_FOR_CRYPTO, DT_POWER_FOR_CRYPTO, 15, false, TIMEOUT_LEFT);

        // Pepare the jewel arm & the optical color/range sensor
        robot.jewelArm.setPosition(robot.JA_MID);
        opMode.sleep(WAIT_FOR_SERVO_MS);

        // Move backward towards cryptobox using optical color/range sensor
        moveTowardsCrypto(DT_POWER_FOR_CRYPTO, DISTANCE_TO_CRYPTO, false, CRS_CRYPTO_TIMEOUT);

        // Rest of the code is similar to Blue near
    }
}
