package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Krishna Saxena on 10/5/2016.
 */
public class LeagueAutoRedBeacon extends BoKAutoCommon {

    private static final double MOVE_FORWARD_FROM_WALL = 10.5; // in inches
    private static final double INITIAL_TURN_ANGLE     = 46;   // in degrees (to the left)
    private static final double TURN_ANGLE_TO_WHITE    = 45;   // in degrees (to the left)
    private static final double MOVE_FORWARD_TO_LINE   = 42.0; // in inches
    private static final double TURN_ANGLE_FOR_BEACON  = 90;   // in degrees (to the left)
    private static final double TURN_ANGLE_FOR_PARK    = -175;

    @Override
    public void initSoftware(LinearOpMode opMode, BoKHardwareBot robot, BoKAlliance redOrBlue) {
        super.initSoftware(opMode, robot, redOrBlue);
    }

    @Override
    public void runSoftware(LinearOpMode opMode, BoKHardwareBot robot) throws InterruptedException
    {
        double shooterMotorsPower = BoKHardwareBot.SHOOTER_MOTORS_POWER_NORMAL;
        double voltage12V = robot.voltageSensor.getVoltage();
        Log.v("BOK", "Battery voltage: " + voltage12V);
        if (voltage12V >= ROBOT_BATTERY_LEVEL_HIGH_THRESHOLD) {
            shooterMotorsPower = BoKHardwareBot.SHOOTER_MOTORS_POWER_NORMAL - SHOOTER_MOTOR_POWER_CHANGE;
        }
        else if (voltage12V < ROBOT_BATTERY_LEVEL_MED_THRESHOLD) {
            shooterMotorsPower = BoKHardwareBot.SHOOTER_MOTORS_POWER_NORMAL + SHOOTER_MOTOR_POWER_CHANGE;
        }

        // First shoot the two balls by turning on the sweeper and the ball shooter
        shootBall(opMode, robot, shooterMotorsPower, WAIT_FOR_SEC_SHOOTER);

        // Move forward using encoders
        moveForward(opMode, robot, LEFT_MOTOR_POWER/POWER_REDUCTION_FACTOR_FWD, RIGHT_MOTOR_POWER/POWER_REDUCTION_FACTOR_FWD, MOVE_FORWARD_FROM_WALL, TWO_SECONDS);
        // Turn using gyro
        gyroTurn(opMode, robot, LEFT_MOTOR_POWER/2, INITIAL_TURN_ANGLE);
        opMode.sleep(SLEEP_100_MS);

        moveForward(opMode, robot, LEFT_MOTOR_POWER/POWER_REDUCTION_FACTOR_FWD, RIGHT_MOTOR_POWER/POWER_REDUCTION_FACTOR_FWD, MOVE_FORWARD_TO_LINE, THREE_SECONDS);
        opMode.sleep(SLEEP_250_MS);

        gyroTurn(opMode, robot, LEFT_MOTOR_POWER/2, TURN_ANGLE_TO_WHITE);

        // Run to white
        if (runToWhite(opMode, robot, THREE_SECONDS) == false) {
            super.exitSoftware();
        }
        opMode.sleep(SLEEP_250_MS);

        turnToWhite(opMode, robot, true/*left*/, TWO_SECONDS);
        opMode.sleep(SLEEP_250_MS);

        // Give enough time for the robot to straighten up using proportional line following
        proportionalLineFollower(opMode, robot, false/* right */, ROBOT_DISTANCE_FROM_WALL_INITIAL);

        gyroTurn(opMode, robot, LEFT_MOTOR_POWER/POWER_REDUCTION_FACTOR_TURN, TURN_ANGLE_FOR_BEACON);
        opMode.sleep(SLEEP_250_MS);

        if (goBackTillBeaconIsVisible(opMode, robot, FOUR_SECONDS)) {
            // Beacon is visible, go forward towards fall
            goForwardToWall(opMode, robot, ROBOT_DISTANCE_FROM_WALL_FOR_BEACON, TWO_SECONDS);
            // Give enough time for the robot to straighten up using proportional line following
            proportionalLineFollower(opMode, robot, false /*right edge*/, ROBOT_DISTANCE_FROM_WALL_FOR_BEACON);
            // Turn using gyro
            gyroTurn(opMode, robot, LEFT_MOTOR_POWER /POWER_REDUCTION_FACTOR_TURN, TURN_ANGLE_FOR_BEACON);

            opMode.sleep(SLEEP_100_MS);

            // Go forward, hit the beacon, come back
            goForwardTillBeacon(opMode, robot, ROBOT_DISTANCE_TO_PUSH_THE_BEACON, TWO_SECONDS);
            goBackFromWall(opMode, robot, ROBOT_DISTANCE_FROM_WALL_AFTER_BEACON, ONE_SECOND);

            // turn for parking
            gyroTurn(opMode, robot, LEFT_MOTOR_POWER, TURN_ANGLE_FOR_PARK);
            // move forward to park
            moveForward(opMode, robot, LEFT_MOTOR_POWER, RIGHT_MOTOR_POWER, MOVE_FORWARD_TO_PARK, FOUR_SECONDS);
        }

        super.exitSoftware();
    }
}
