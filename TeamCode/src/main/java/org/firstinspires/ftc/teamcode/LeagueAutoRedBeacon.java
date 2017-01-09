package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Krishna Saxena on 10/5/2016.
 */
public class LeagueAutoRedBeacon extends BoKAutoCommon {

    protected static final double WAIT_FOR_SEC_SHOOTER = 5.0;
    protected static final int ROBOT_DISTANCE_FROM_WALL_FOR_BEACON = 16;
    protected static final int ROBOT_DISTANCE_FROM_WALL_AFTER_BEACON = 32;

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
        if (voltage12V >= 13.0) {
            shooterMotorsPower = BoKHardwareBot.SHOOTER_MOTORS_POWER_NORMAL - 0.1;
        }
        else if (voltage12V < 12.4) {
            shooterMotorsPower = BoKHardwareBot.SHOOTER_MOTORS_POWER_NORMAL + 0.1;
        }

        // First shoot the two balls by turning on the sweeper and the ball shooter
        shootBall(opMode, robot, shooterMotorsPower, WAIT_FOR_SEC_SHOOTER);

        // Move forward for 8 inch in 1.5 sec
        moveForward(opMode, robot, LEFT_MOTOR_POWER/1.5, RIGHT_MOTOR_POWER/1.5, 10.5, 2);
        // Turn 35 degrees (in 0.5 sec or less)
        gyroTurn(opMode, robot, LEFT_MOTOR_POWER/2, 46);
        opMode.sleep(100);

        moveForward(opMode, robot, LEFT_MOTOR_POWER/1.5, RIGHT_MOTOR_POWER/1.5, 42.0, 3);
        opMode.sleep(250);

        gyroTurn(opMode, robot, LEFT_MOTOR_POWER/2, 45);

        // Run to white
        if (runToWhite(opMode, robot, 4/*sec*/) == false) {
            super.exitSoftware();
        }
        opMode.sleep(250);

        turnToWhite(opMode, robot, true/*left*/, 2/*sec*/);
        opMode.sleep(250);

        proportionalLineFollower(opMode, robot, false/* right */, 15); // 15 cm; give enough time for the robot to straighten up

        gyroTurn(opMode, robot, LEFT_MOTOR_POWER/2.5, 90);
        opMode.sleep(250);

        if (goBackTillBeaconIsVisible(opMode, robot, 4/*sec*/)) {

            goForwardToWall(opMode, robot, ROBOT_DISTANCE_FROM_WALL_FOR_BEACON, 2/*sec*/); // 8 cm
            proportionalLineFollower(opMode, robot, false /*right edge*/, ROBOT_DISTANCE_FROM_WALL_FOR_BEACON);  // 8 cm
            gyroTurn(opMode, robot, LEFT_MOTOR_POWER /2.5, 90);

            opMode.sleep(100);

            goForwardTillBeacon(opMode, robot, 9, 2/*sec*/); // 8 cm
            goBackFromWall(opMode, robot, ROBOT_DISTANCE_FROM_WALL_AFTER_BEACON, 0.5);

            gyroTurn(opMode, robot, LEFT_MOTOR_POWER, -175);
            moveForward(opMode, robot, LEFT_MOTOR_POWER, RIGHT_MOTOR_POWER, 45.0, 4);
        }
        else {
            opMode.sleep(15000);
        }
        super.exitSoftware();
    }
}
