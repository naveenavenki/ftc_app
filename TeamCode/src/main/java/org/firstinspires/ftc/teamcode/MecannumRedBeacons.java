package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Krishna Saxena on 10/5/2016.
 * Implements runSoftware method so that we can move the robot when alliance color is blue
 */
public class MecannumRedBeacons extends BoKMecanumAutoCommon {

    private static final double MOVE_FORWARD_FROM_WALL = 10.5; // in inches
    private static final double INITIAL_TURN_ANGLE     = 44;   // in degrees (to the left)
    private static final double TURN_ANGLE_TO_WHITE    = 44;   // in degrees (to the left)
    private static final double MOVE_FORWARD_TO_LINE   = 42.0; // in inches
    private static final double TURN_ANGLE_FOR_BEACON  = 89;   // in degrees (to the left)
    private static final double TURN_ANGLE_FOR_PARK    = -179;

    @Override
    public void initSoftware(LinearOpMode opMode, BoKHardwareBot robot, BoKAlliance redOrBlue) {
        super.initSoftware(opMode, robot, redOrBlue);
    }

    public void runSoftware(LinearOpMode opMode, BoKHardwareBot robot)
    {
        //double shooterMotorsPower = getShooterMotorsPowerBasedOnBatteryLevel(robot);
        alignToWall(opMode,robot,30.0,10.0);
        //robot.gyroSensor.resetZAxisIntegrator();

        //gyroTurn(opMode, robot,LEFT_MOTOR_POWER,45);
        //opMode.sleep(100);
        runToWhiteSideways(opMode, robot, 5.0);

        // detect beacom color
        detectBeaconColor(opMode, robot);

    }

}
