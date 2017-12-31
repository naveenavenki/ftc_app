package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoBlueNear extends BoKAutoCommon
{
    private static final double TIMEOUT_LEFT = 4;
    private static final double TIMEOUT_CENTER = 5;
    private static final double TIMEOUT_RIGHT = 6;

    private static final double DISTANCE_TO_LEFT_COL = 21; // inches!!
    private static final double DISTANCE_TO_CENTER_COL = 29;
    private static final double DISTANCE_TO_RIGHT_COL = 37;

    // Constructor
    public BoKAutoBlueNear()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_BLUE;
    }

    @Override
    public void runSoftware()
    {
        // NOTE: Move backwards towards crypto

        // Detect Vuforia image and flick the jewel
        Log.v("BOK", "RSF " + robot.rangeSensorFront.getDistance(DistanceUnit.CM));
        detectVuforiaImgAndFlick();

        // Move forward out of balancing stone
        // Distance and timeout depends on column number; TBD
        double distance = DISTANCE_TO_LEFT_COL;
        double timeout = TIMEOUT_LEFT;
        if (cryptoColumn == RelicRecoveryVuMark.CENTER) {
            distance = DISTANCE_TO_CENTER_COL;
            timeout = TIMEOUT_CENTER;
        }
        else if (cryptoColumn == RelicRecoveryVuMark.RIGHT) {
            distance = DISTANCE_TO_RIGHT_COL;
            timeout = TIMEOUT_RIGHT;
        }

        // Move backward out of balancing stone
        // Distance and timeout depends on column number;
        move(DT_POWER_FOR_STONE,
             DT_POWER_FOR_STONE,
             distance,
             false,
             timeout);
        Log.v("BOK", "RSF " + robot.rangeSensorFront.getDistance(DistanceUnit.CM));
        // Strafe to the right
        //strafe(DT_POWER_FOR_STRAFE,
        //       ROTATIONS_STRAFE_TO_WALL,
        //      true,
        //       DT_STRAFE_TIMEOUT);

        moveToCrypto();
        gyroTurn(DT_TURN_SPEED_HIGH, 0, TURN_LEFT_DEGREES, DT_TURN_TIMEOUT);
        move(DT_POWER_FOR_STONE, DT_POWER_FOR_STONE, 10, false, DT_TIMEOUT);
        moveGlyphFlicker();
        move(DT_POWER_FOR_STONE, DT_POWER_FOR_STONE, 4, true, DT_TIMEOUT);

    }
}
