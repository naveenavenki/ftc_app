package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoRedFar extends BoKAutoCommon {
    private static double TIMEOUT_RIGHT = 4;
    private static double TIMEOUT_CENTER = 5;
    private static double TIMEOUT_LEFT = 6;
    private static double DT_MOVE_TO_CRYPTO = 26;//inches
    private static int DISTANCE_TO_CENTER_COL_CM = 70;//43;//cm
    private static int DISTANCE_TO_RIGHT_COL_CM = 56;//24;//cm
    private static int DISTANCE_TO_LEFT_COL_CM = 95;//cm

    // Constructor
    public BoKAutoRedFar()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_RED;
    }

    @Override
    public void runSoftware()
    {
        far = true;

        // Detect Vuforia image, flick the jewel
        detectVuforiaImgAndFlick();

        // Move out of the balancing stone, distance: TBD?
        move(DT_POWER_FOR_STONE, DT_POWER_FOR_STONE, DT_MOVE_TO_CRYPTO, true, DT_TIMEOUT);

        // turn left 90 degrees
        double current_angle = gyroTurn(DT_TURN_SPEED_HIGH, 0, TURN_LEFT_DEGREES, DT_TURN_TIMEOUT);

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

        //turn left 90 degrees
        gyroTurn(DT_TURN_SPEED_HIGH, current_angle, 180, DT_TURN_TIMEOUT*3);

        move(DT_POWER_FOR_STONE, DT_POWER_FOR_STONE, 7.5, false, DT_TIMEOUT);

        moveGlyphFlicker();

        move(DT_POWER_FOR_STONE, DT_POWER_FOR_STONE, 4, true, DT_TIMEOUT);
    }

}
