package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Krishna Saxena on 10/5/2016.
 */
public class League1AutoRed extends BoKAutoCommon {

    protected static final double WAIT_FOR_SEC_SHOOTER = 8.0;
    //private static final double WAIT_FOR_SEC_LINE = 4.0;
    protected static final int ROBOT_DISTANCE_FROM_WALL_FOR_BEACON = 10;
    protected static final int ROBOT_DISTANCE_FROM_WALL_AFTER_BEACON = 20;

    @Override
    public void initSoftware(LinearOpMode opMode, BoKHardwareBot robot, BoKAlliance redOrBlue) {
        super.initSoftware(opMode, robot, redOrBlue);
    }

    @Override
    public void runSoftware(LinearOpMode opMode, BoKHardwareBot robot) throws InterruptedException
    {
        // First shoot the two balls by turning on the sweeper and the ball shooter
        shootBall(opMode, robot, BoKHardwareBot.SHOOTER_MOTORS_POWER, WAIT_FOR_SEC_SHOOTER);

        // Move forward for 8 inch in 1.5 sec
        moveForward(opMode, robot, 8.0, 4);
        // Turn 35 degrees (in 0.5 sec or less)
        gyroTurn(opMode, robot, LEFT_MOTOR_POWER/3, 45);
        opMode.sleep(100);
        // moveTurn(opMode, robot, 45.0, true, 0.5);

        // Run to white
        runToWhite(opMode, robot, 5/*sec*/);
        opMode.sleep(250);
        //runToGray(opMode, robot, 1);
        turnToWhite(opMode, robot, true/*left*/, 2/*sec*/);
        opMode.sleep(250);
        proportionalLineFollower(opMode, robot, false/* right */, 15); // 15 cm; give enough time for the robot to straighten up

        gyroTurn(opMode, robot, LEFT_MOTOR_POWER/3, 90);
        opMode.sleep(250);

        goBackTillBeaconIsVisible(opMode, robot, 2/*sec*/);

        goForwardToWall(opMode, robot, ROBOT_DISTANCE_FROM_WALL_FOR_BEACON, 2/*sec*/); // 8 cm
        proportionalLineFollower(opMode, robot, false /*right edge*/, ROBOT_DISTANCE_FROM_WALL_FOR_BEACON);  // 8 cm
        goBackFromWall(opMode, robot, ROBOT_DISTANCE_FROM_WALL_AFTER_BEACON, 0.5);

        super.exitSoftware();
    }
}
