package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoRedFar extends BoKAutoCommon {
    private static double DT_TIMEOUT = 4;
    private static double TIMEOUT_RIGHT = 4;
    private static double TIMEOUT_CENTER = 5;
    private static double TIMEOUT_LEFT = 6;
    private static int TURN_LEFT_DEGREES = 90;
    private static double DT_MOVE_TO_CRYPTO = 24.75;//inches
    private static int DISTANCE_TO_CENTER_COL_CM = 43;//cm
    private static int DISTANCE_TO_RIGHT_COL_CM = 25;//cm
    private static int DISTANCE_TO_LEFT_COL_CM = 63;//cm

    @Override
    public void runSoftware()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_RED;

        // Detect Vuforia image, flick the jewel
        detectVuforiaImgAndFlick();

        // Move out of the balancing stone, distance: TBD?
        move(DT_POWER_FOR_STONE, DT_POWER_FOR_STONE, DT_MOVE_TO_CRYPTO, true, DT_TIMEOUT);

        // turn left 90 degrees
        gyroTurn(DT_TURN_SPEED_HIGH, TURN_LEFT_DEGREES, DT_TURN_TIMEOUT);

        // Move backwards to wall: TBD? (if there isn't enough space)

        // Move forwards towards cryptobox
        // Distance and timeout depends on column number
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
        moveWithRangeSensor(DT_POWER_FOR_RS, distance, false, timeout); // CM

        // Prepare to unload the glyph
        moveToCrypto();
    }
}
