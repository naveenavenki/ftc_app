package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Krishna Saxena on 10/5/2016.
 * Implements runSoftware method so that we can move the robot when alliance color is red
 */
public class LeagueAutoBlueBeacon extends BoKAutoCommon {

    private static final double MOVE_FORWARD_FROM_WALL = 12; // in inches
    private static final double INITIAL_TURN_ANGLE    = -43; // in degrees (to the right)
    private static final double TURN_ANGLE_TO_WHITE   = -44; // in degrees (to the right)
    private static final double MOVE_FORWARD_TO_LINE  = 42.0;// in inches
    private static final double TURN_ANGLE_FOR_BEACON = -89; // in degrees (to the right)
    private static final double TURN_ANGLE_FOR_PARK   = -175;

    @Override
    public void initSoftware(LinearOpMode opMode, BoKHardwareBot robot, BoKAlliance redOrBlue) {
        super.initSoftware(opMode, robot, redOrBlue);
    }

    @Override
    public void runSoftware(LinearOpMode opMode, BoKHardwareBot robot)
    {
        double shooterMotorsPower = getShooterMotorsPowerBasedOnBatteryLevel(robot);

        // First shoot the two balls by turning on the sweeper and the ball shooter
        shootBall(opMode, robot, shooterMotorsPower, LeagueAutoRedBeacon.WAIT_FOR_SEC_SHOOTER);

        // Move forward using encoder
        moveForward(opMode, robot,
                LEFT_MOTOR_POWER/POWER_REDUCTION_FACTOR_FWD,
                RIGHT_MOTOR_POWER/POWER_REDUCTION_FACTOR_FWD, MOVE_FORWARD_FROM_WALL, TWO_SECONDS);

        // Turn using gyro
        gyroTurn(opMode, robot, LEFT_MOTOR_POWER/POWER_REDUCTION_FACTOR_TURN, INITIAL_TURN_ANGLE);
        opMode.sleep(SLEEP_100_MS);

        // Move forward using encoder
        moveForward(opMode, robot,
                LEFT_MOTOR_POWER/POWER_REDUCTION_FACTOR_FWD,
                RIGHT_MOTOR_POWER/POWER_REDUCTION_FACTOR_FWD, MOVE_FORWARD_TO_LINE, THREE_SECONDS);
        opMode.sleep(SLEEP_250_MS);

        gyroTurn(opMode, robot, LEFT_MOTOR_POWER/2, TURN_ANGLE_TO_WHITE);

        // Run to white
        if (runToWhite(opMode, robot, THREE_SECONDS) == false) {
            super.exitSoftware();
        }
        opMode.sleep(SLEEP_250_MS);

        turnToWhite(opMode, robot, false/*right*/, ONE_SECOND);
        opMode.sleep(SLEEP_250_MS);

        // Give enough time for the robot to straighten up using proportional line following
        proportionalLineFollower(opMode, robot, true /*left */, ROBOT_DISTANCE_FROM_WALL_INITIAL);
        opMode.sleep(SLEEP_250_MS);

        gyroTurn(opMode, robot,
                LEFT_MOTOR_POWER/POWER_REDUCTION_FACTOR_TURN, TURN_ANGLE_FOR_BEACON);
        opMode.sleep(SLEEP_250_MS);

        if (goBackTillBeaconIsVisible(opMode, robot, FOUR_SECONDS)) {
            // Beacon is visible, go forward towards fall
            goForwardToWall(opMode, robot, ROBOT_DISTANCE_FROM_WALL_FOR_BEACON, TWO_SECONDS);

            // Give enough time for the robot to straighten up using proportional line following
            proportionalLineFollower(opMode, robot,
                    true/*left edge*/, ROBOT_DISTANCE_FROM_WALL_FOR_BEACON);

            // Turn using gyro
            gyroTurn(opMode, robot,
                    LEFT_MOTOR_POWER/POWER_REDUCTION_FACTOR_TURN, TURN_ANGLE_FOR_BEACON);
            opMode.sleep(SLEEP_100_MS);

            // Go forward, hit the beacon, come back
            goForwardTillBeacon(opMode, robot, ROBOT_DISTANCE_TO_PUSH_THE_BEACON, TWO_SECONDS);
            goBackFromWall(opMode, robot, ROBOT_DISTANCE_FROM_WALL_AFTER_BEACON, ONE_SECOND);

            // turn for parking
            gyroTurn(opMode, robot, LEFT_MOTOR_POWER, TURN_ANGLE_FOR_PARK);
            // move forward to park
            moveForward(opMode, robot,
                    LEFT_MOTOR_POWER*2, RIGHT_MOTOR_POWER*2, MOVE_FORWARD_TO_PARK, FOUR_SECONDS);
        } // if goBackTillBeaconIsVisible

        super.exitSoftware();
    }
}
