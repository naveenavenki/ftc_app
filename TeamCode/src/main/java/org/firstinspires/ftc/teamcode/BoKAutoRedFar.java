package org.firstinspires.ftc.teamcode;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoRedFar extends BoKAutoCommon {
    private static double DT_TIMEOUT = 4;
    private static double TIMEOUT_RIGHT = 4;
    private static double TIMEOUT_CENTER = 5;
    private static double TIMEOUT_LEFT = 6;
    private static int TURN_LEFT_DEGREES = 90;
    private static double DT_MOVE_TO_CRYPTO = 16;

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
        //move(DT_POWER_FOR_CRYPTO, DT_POWER_FOR_STONE, 15, false, TIMEOUT_RIGHT);

        // prepare the jewel arm & the optical color/range sensor

        // move forward towards cryptobox using optical color/range sensor
        
        moveWithRangeSensor(0.15, 25, false, 4);
    
      //  moveTowardsCrypto(DT_POWER_FOR_CRYPTO, DISTANCE_TO_CRYPTO, true, CRS_CRYPTO_TIMEOUT);

        // Rest of the code is similar to Red near: TBD
    }
}
