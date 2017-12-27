package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoRedNear extends BoKAutoCommon {
    private static final double TIMEOUT_RIGHT = 4;
    private static final double TIMEOUT_CENTER = 5;
    private static final double TIMEOUT_LEFT = 6;
    
    private static final double DISTANCE_TO_RIGHT_COL = 16.5; // inches!!
    private static final double DISTANCE_TO_CENTER_COL = 26;
    private static final double DISTANCE_TO_LEFT_COL = 33;

    // Constructor
    public BoKAutoRedNear()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_RED;
    }

    @Override
    public void runSoftware()
    {
       /* robot.glyphArm.clawGrab.setPosition(robot.CG_OPEN);
        opMode.sleep(WAIT_FOR_SERVO_MS);
        robot.glyphArm.clawWrist.setPosition(0.4);
        opMode.sleep(WAIT_FOR_SERVO_MS);
        robot.glyphArm.clawGrab.setPosition(robot.CG_CLOSE);
        opMode.sleep(WAIT_FOR_SERVO_MS);

        //arm position in encoder counts : 1902
        robot.glyphArm.moveUpperArm(122,0.2);
        robot.glyphArm.clawWrist.setPosition(0.88);
        robot.glyphArm.clawGrab.setPosition(robot.CG_OPEN);
        //robot.glyphArm.moveUpperArm(-155,-0.2);
*/
        // Detect Vuforia image and flick the jewel
        detectVuforiaImgAndFlick();

        // Move forward out of balancing stone
        // Distance and timeout depends on column number; TBD
        double distance = DISTANCE_TO_RIGHT_COL;
        double timeout = TIMEOUT_RIGHT;
        if (cryptoColumn == RelicRecoveryVuMark.CENTER) {
            distance = DISTANCE_TO_CENTER_COL;
            timeout = TIMEOUT_CENTER;
        } else if (cryptoColumn == RelicRecoveryVuMark.LEFT) {
            distance = DISTANCE_TO_LEFT_COL;
            timeout = TIMEOUT_LEFT;
        }

        move(DT_POWER_FOR_STONE,
             DT_POWER_FOR_STONE,
             distance,
             true,
             timeout);

        // Strafe to the right
        //strafe(DT_POWER_FOR_STRAFE,
        //       ROTATIONS_STRAFE_TO_WALL,
        //       true,
        //       DT_STRAFE_TIMEOUT);

        // Prepare to unload the glyph
        moveToCrypto();

        gyroTurn(0.2,-90,5.0);
    }


}
