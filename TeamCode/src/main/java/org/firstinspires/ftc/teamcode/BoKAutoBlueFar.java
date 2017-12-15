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
    private static int DISTANCE_TO_LEFT_COL_CM = 47;
    private static int DISTANCE_TO_CENTER_COL_CM = 60;
    private static int DISTANCE_TO_RIGHT_COL_CM = 78;

    // Constructor
    public BoKAutoBlueFar()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_BLUE;
    }

    @Override
    public void runSoftware()
    {
        // NOTE: Move backwards towards crypto

        // Detect Vuforia image and flick the jewel
        detectVuforiaImgAndFlick();

        // Move back out of the balancing stone
        move(DT_POWER_FOR_STONE, DT_POWER_FOR_STONE, DT_MOVE_TO_CRYPTO, false, DT_TIMEOUT);

        // Turn right 90 degrees
        gyroTurn(DT_TURN_SPEED_HIGH, TURN_RIGHT_DEGREES, DT_TURN_TIMEOUT);

        // Distance to crypto and timeout depends on column number
        int distance = DISTANCE_TO_RIGHT_COL_CM;
        double timeout = TIMEOUT_RIGHT;
        if (cryptoColumn == RelicRecoveryVuMark.CENTER) {
            distance = DISTANCE_TO_CENTER_COL_CM;
            timeout = TIMEOUT_CENTER;
        }
        else if (cryptoColumn == RelicRecoveryVuMark.LEFT) {
            distance = DISTANCE_TO_LEFT_COL_CM;
            timeout = TIMEOUT_LEFT;
        }

        // Move towards the crypto
        moveWithRangeSensor(DT_POWER_FOR_RS, distance, true, timeout); // CM

        // Prepare to unload the glyph
        moveToCrypto();
    }
}
