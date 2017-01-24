package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Krishna Saxena on 10/5/2016.
 * Implements runSoftware method so that we can move the robot when alliance color is blue
 */
public class LeagueAutoFar extends BoKAutoCommon {



    @Override
    public void initSoftware(LinearOpMode opMode, BoKHardwareBot robot, BoKAlliance redOrBlue) {
        super.initSoftware(opMode, robot, redOrBlue);
    }

    @Override
    public void runSoftware(LinearOpMode opMode, BoKHardwareBot robot)
    {
        opMode.sleep(8000);
        double shooterMotorsPower = getShooterMotorsPowerBasedOnBatteryLevel(robot);
        // Move forward using encoders
        moveForward(opMode, robot,
                LEFT_MOTOR_POWER/POWER_REDUCTION_FACTOR_FWD,
                RIGHT_MOTOR_POWER/POWER_REDUCTION_FACTOR_FWD, 22, 5);

        // First shoot the two balls by turning on the sweeper and the ball shooter
        shootBall(opMode, robot, shooterMotorsPower, WAIT_FOR_SEC_SHOOTER);

        robot.sweeperMotor.setPower(-0.9);
        // Move forward using encoders
        moveForward(opMode, robot,
                LEFT_MOTOR_POWER/POWER_REDUCTION_FACTOR_FWD,
                RIGHT_MOTOR_POWER/POWER_REDUCTION_FACTOR_FWD, 60, 5);

        super.exitSoftware();
    }
}
