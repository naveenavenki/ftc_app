package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Krishna Saxena on 10/5/2016.
 */
public class League1AutoRed extends BoKAutoCommon {

    private static final double WAIT_FOR_SEC_SHOOTER = 8.0;
    private static final double WAIT_FOR_SEC_LINE = 4.0;

    @Override
    public void initSoftware(LinearOpMode opMode, BoKHardwareBot robot, BoKAlliance redOrBlue) {
        super.initSoftware(opMode, robot, redOrBlue);
    }

    @Override
    public void runSoftware(LinearOpMode opMode, BoKHardwareBot robot) throws InterruptedException
    {
        // First shoot the two balls by turning on the sweeper and the ball shooter
        //shootBall(opMode, robot, BoKHardwareBot.SHOOTER_MOTORS_POWER, WAIT_FOR_SEC_SHOOTER);

        // Move forward for 8 inch in 1.5 sec
        moveForward(opMode, robot, 8.0, 1.5);
        // Turn 35 degrees (in 0.5 sec or less)
        moveTurn(opMode, robot, 45.0, true, 0.5);

        // Run to white
        runToWhite(opMode, robot, 5/*sec*/);
        turnToWhite(opMode, robot, true/*left*/, 1/*sec*/);
        proportionalLineFollower(opMode, robot, 15); // 15 cm; give enough time for the robot to straighten up

        goBackTillBeaconIsVisible(opMode, robot, 2/*sec*/);

        goForwardToWall(opMode, robot, 50, 4/*sec*/); // 50 cm
        proportionalLineFollower(opMode, robot, 8);  // 8 cm
        goBackFromWall(opMode, robot, 20, 0.5);

        super.exitSoftware();
    }
}
