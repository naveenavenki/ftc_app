package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoBlueFar extends BoKAutoCommon
{
    private static double DT_TIMEOUT = 4;
    private static double TIMEOUT_LEFT = 6;
    private static double TIMEOUT_CENTER = 8;
    private static double TIMEOUT_RIGHT = 10;
    private static int TURN_RIGHT_DEGREES = -90;
    private static double DT_MOVE_TO_CRYPTO = 19;
    private static int DISTANCE_TO_LEFT_COL_CM = 32 ;
    private static int DISTANCE_TO_CENTER_COL_CM = 51;
    private static int DISTANCE_TO_RIGHT_COL_CM = 70;

    // Constructor
    public BoKAutoBlueFar()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_BLUE;
    }

    @Override
    public void runSoftware()
    {
        // NOTE: Move backwards towards crypto
        far = true;

        // Detect Vuforia image and flick the jewel
        detectVuforiaImgAndFlick();

        // Move back out of the balancing stone
        move(DT_POWER_FOR_STONE, DT_POWER_FOR_STONE, DT_MOVE_TO_CRYPTO, false, DT_TIMEOUT);

        // Turn right 90 degrees
        double current_angle = gyroTurn(DT_TURN_SPEED_HIGH, 0, TURN_RIGHT_DEGREES, DT_TURN_TIMEOUT);

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

        //turn left 90 degrees
        gyroTurn(DT_TURN_SPEED_HIGH, current_angle, 0, DT_TURN_TIMEOUT*3);

        move(DT_POWER_FOR_STONE, DT_POWER_FOR_STONE, 7.5, false, DT_TIMEOUT);

        moveGlyphFlicker();

        move(DT_POWER_FOR_STONE, DT_POWER_FOR_STONE, 4, true, DT_TIMEOUT);

    }
}
