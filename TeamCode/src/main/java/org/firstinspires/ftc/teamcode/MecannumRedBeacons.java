package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Krishna Saxena on 10/5/2016.
 * Implements runSoftware method so that we can move the robot when alliance color is blue
 */
public class MecannumRedBeacons extends BoKMecanumAutoCommon {

    private static final double MOVE_FORWARD_FROM_WALL = 10.5; // in inches
    private static final double INITIAL_TURN_ANGLE     = 30;   // in degrees (to the left)
    private static final double TURN_ANGLE_TO_WHITE    = 75;   // in degrees (to the left)
    private static final double MOVE_FORWARD_TO_LINE   = 75; // in inches
    private static final double TURN_ANGLE_FOR_BEACON  = 89;   // in degrees (to the left)
    private static final double TURN_ANGLE_FOR_PARK    = -179;

    @Override
    public void initSoftware(LinearOpMode opMode, BoKHardwareBot robot, BoKAlliance redOrBlue) {
        super.initSoftware(opMode, robot, redOrBlue);
    }

    public void runSoftware(LinearOpMode opMode, BoKHardwareBot robot)
    {
        double shooterMotorsPower = getShooterMotorsPowerBasedOnBatteryLevel(robot);

        // First shoot the two balls by turning on the sweeper and the ball shooter
        //shootBall(opMode, robot, shooterMotorsPower, WAIT_FOR_SEC_SHOOTER);

        // Move forward using encoders
/*
        moveForward(opMode, robot,
                LEFT_MOTOR_POWER,
                RIGHT_MOTOR_POWER, MOVE_FORWARD_FROM_WALL, TWO_SECONDS);

        robot.gyroSensor.resetZAxisIntegrator();
        gyroTurn(opMode, robot,LEFT_MOTOR_POWER, INITIAL_TURN_ANGLE);

        moveForward(opMode, robot,
                LEFT_MOTOR_POWER,
                RIGHT_MOTOR_POWER, MOVE_FORWARD_TO_LINE, FOUR_SECONDS);

        gyroTurn(opMode, robot,LEFT_MOTOR_POWER, TURN_ANGLE_TO_WHITE);

        alignToWall(opMode, robot, 30.0, RIGHT_MOTOR_POWER/2, true, 4);
        alignToWall(opMode, robot, 30.0, LEFT_MOTOR_POWER/2, false, 4);

        //alignToWall(opMode, robot, 30.0, RIGHT_MOTOR_POWER/2, true, 4);
        //alignToWall(opMode, robot, 30.0, LEFT_MOTOR_POWER/2, false, 4);

        //alignToWall(opMode,robot,30.0,10.0);
        //opMode.sleep(100);
        runToWhiteSideways(opMode, robot, LEFT_MOTOR_POWER, RIGHT_MOTOR_POWER, true, 5.0);
*/
        // detect beacom color
        detectBeaconColor(opMode, robot, 0, 2.0);

        //slideSideways(opMode, robot, LEFT_MOTOR_POWER, RIGHT_MOTOR_POWER, false, 3);

        //runParallelToWall(opMode, robot, LEFT_MOTOR_POWER/2, RIGHT_MOTOR_POWER/2, 30, true, 5);
        //alignToWall(opMode, robot, 30.0, RIGHT_MOTOR_POWER/2, true, 4);
        //alignToWall(opMode, robot, 30.0, LEFT_MOTOR_POWER/2, false, 4);

        //alignToWall(opMode, robot, 30.0, RIGHT_MOTOR_POWER/2, true, 4);
        //alignToWall(opMode, robot, 30.0, LEFT_MOTOR_POWER/2, false, 4);

        //runToWhiteSideways(opMode, robot, LEFT_MOTOR_POWER, RIGHT_MOTOR_POWER, false, 5.0);
        //detectBeaconColor(opMode, robot, 1, 2.0);
    }

}
