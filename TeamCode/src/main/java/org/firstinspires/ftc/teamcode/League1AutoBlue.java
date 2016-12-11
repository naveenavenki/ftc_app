package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Krishna Saxena on 10/5/2016.
 */
public class League1AutoBlue extends BoKAutoCommon {

    @Override
    public void initSoftware(LinearOpMode opMode, BoKHardwareBot robot, BoKAlliance redOrBlue) {
        super.initSoftware(opMode, robot, redOrBlue);
    }

    @Override
    public void runSoftware(LinearOpMode opMode, BoKHardwareBot robot) throws InterruptedException
    {
        // First shoot the two balls by turning on the sweeper and the ball shooter
        shootBall(opMode, robot, BoKHardwareBot.SHOOTER_MOTORS_POWER-0.1, League1AutoRed.WAIT_FOR_SEC_SHOOTER);

        // Move forward for 8 inch in 1.5 sec
        moveForward(opMode, robot, 9.0, 1.5);
        // Turn 35 degrees (in 0.5 sec or less)
        gyroTurn(opMode, robot, LEFT_MOTOR_POWER/3, -46);
        opMode.sleep(100);

        moveForward(opMode, robot, 35.0, 4);
        opMode.sleep(250);

        gyroTurn(opMode, robot, LEFT_MOTOR_POWER/2.5, -46);

        // Run to white
        runToWhite(opMode, robot, 4/*sec*/);
        opMode.sleep(250);

        //runToGray(opMode, robot, 1);
        turnToWhite(opMode, robot, false/*right*/, 1/*sec*/);
        opMode.sleep(250);

        proportionalLineFollower(opMode, robot, true /*left */, 15); // 15 cm; give enough time for the robot to straighten up
        opMode.sleep(250);

        gyroTurn(opMode, robot, LEFT_MOTOR_POWER/3, -92);
        opMode.sleep(250);

        if (goBackTillBeaconIsVisible(opMode, robot, 4/*sec*/)) {

            goForwardToWall(opMode, robot, League1AutoRed.ROBOT_DISTANCE_FROM_WALL_FOR_BEACON, 2/*sec*/); // 8 cm

            proportionalLineFollower(opMode, robot, true/*left*/, League1AutoRed.ROBOT_DISTANCE_FROM_WALL_FOR_BEACON);  // 8 cm
            gyroTurn(opMode, robot, LEFT_MOTOR_POWER / 3, -92);
            opMode.sleep(100);

            goForwardTillBeacon(opMode, robot, 9, 2/*sec*/); // 8 cm
            goBackFromWall(opMode, robot, League1AutoRed.ROBOT_DISTANCE_FROM_WALL_AFTER_BEACON, 0.5);
        }
        else {
            opMode.sleep(15000);
        }
        super.exitSoftware();
    }
}
