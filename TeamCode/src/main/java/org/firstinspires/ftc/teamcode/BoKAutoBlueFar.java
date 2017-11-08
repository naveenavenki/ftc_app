package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

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
    private static double DT_MOVE_TO_CRYPTO = 16.5;
    private static int DISTANCE_TO_LEFT_COL = 44; // cm
    private static int DISTANCE_TO_CENTER_COL = 60; // cm
    private static int DISTANCE_TO_RIGHT_COL = 78; // cm
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

        // Move out of the balancing stone, distance
        move(DT_POWER_FOR_STONE, DT_POWER_FOR_STONE, DT_MOVE_TO_CRYPTO, false, DT_TIMEOUT);

        // turn right 90 degrees
        gyroTurn(DT_TURN_SPEED_HIGH, TURN_RIGHT_DEGREES, DT_TURN_TIMEOUT);

        // Move towards wall: TBD? (if there isn't enough space)

        // Move back towards cryptobox
        // Distance and timeout depends on column number; TBD
        int distance = DISTANCE_TO_RIGHT_COL;
        double timeout = TIMEOUT_RIGHT;
        cryptoColumn = RelicRecoveryVuMark.LEFT;
        if (cryptoColumn == RelicRecoveryVuMark.CENTER) {
            distance = DISTANCE_TO_CENTER_COL;
            timeout = TIMEOUT_CENTER;
        }
        else if (cryptoColumn == RelicRecoveryVuMark.LEFT) {
            distance = DISTANCE_TO_LEFT_COL;
            timeout = TIMEOUT_LEFT;
        }

        
        moveWithRangeSensor(DT_POWER_FOR_RS, distance, true, timeout); // CM

        // Pepare the jewel arm & the optical color/range sensor
        moveBlueCrypto();


        // Move backward towards cryptobox using optical color/range sensor
        //moveTowardsCrypto(DT_POWER_FOR_CRYPTO, DISTANCE_TO_CRYPTO, false, CRS_CRYPTO_TIMEOUT);

        // Rest of the code is similar to Blue near
    }
}
