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
    private static double DT_MOVE_TO_CRYPTO = 24;//inches
    private static int DISTANCE_TO_LEFT_COL = 63;//cm
    private static int DISTANCE_TO_CENTER_COL = 40;//cm
    private static int DISTANCE_TO_RIGHT_COL = 43;//cm

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

        // Move out of the balancing stone, distance: TBD?
        move(DT_POWER_FOR_STONE, DT_POWER_FOR_STONE, DT_MOVE_TO_CRYPTO, true, DT_TIMEOUT);

        // turn left 90 degrees
        gyroTurn(DT_TURN_SPEED_HIGH, TURN_LEFT_DEGREES, DT_TURN_TIMEOUT);

        // Move backwards to wall: TBD? (if there isn't enough space)

        // Move forwards towards cryptobox
        // Distance and timeout depends on column number; TBD
        
        int distance = DISTANCE_TO_RIGHT_COL;
        double timeout = TIMEOUT_RIGHT;
        cryptoColumn = RelicRecoveryVuMark.CENTER;
        if (cryptoColumn == RelicRecoveryVuMark.CENTER) {
            distance = DISTANCE_TO_CENTER_COL;
            timeout = TIMEOUT_CENTER;
        }
        else if (cryptoColumn == RelicRecoveryVuMark.LEFT) {
            distance = DISTANCE_TO_LEFT_COL;
            timeout = TIMEOUT_LEFT;
        }
        

        // prepare the jewel arm & the optical color/range sensor

        // move forward towards cryptobox using optical color/range sensor
        
        moveWithRangeSensor(DT_POWER_FOR_RS, distance, false, timeout);
    
        moveRedCrypto();

        // Rest of the code is similar to Red near: TBD
    }
}
