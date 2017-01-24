package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Krishna Saxena on 10/5/2016.
 */
public class League0Auto implements BoKAuto {

    private static final double WAIT_FOR_SEC_SHOOTER = 8.0;
    private static final double WAIT_FOR_SEC_LINE = 4.0;
    private static final int RED_THRESHOLD  = 5;
    private static final int BLUE_THRESHOLD = 5;

    private static final double SHOOTER_SERVO_POS = 0.1;
    private double positionLeft = BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_LEFT;
    private double positionRight = BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_RIGHT;

    private static final double SHOOTER_MOTOR_POWER = 1.0;
    private static final double SWEEPER_MOTOR_POWER = 1.0;

    private static final int MIDDLE_LINE = 20;

    private ElapsedTime runTime  = new ElapsedTime();
    private double distance;
    private int delta;
    private double steer;
    private int alpha;

    @Override
    public void initSoftware(LinearOpMode opMode, BoKHardwareBot robot, BoKAlliance redOrBlue) {
    }

    @Override
    public void runSoftware(LinearOpMode opMode, BoKHardwareBot robot)
    {
        // set the initial position (both pointed down)
        //robot.setLeftPusherPos(positionLeft);
        //robot.setRightPusherPos(positionRight);
/*
        robot.setShooterServoPos(SHOOTER_SERVO_POS);
        // First shoot the two balls by turning on the sweeper and the ball shooter
        shootBall(opMode, robot, WAIT_FOR_SEC_SHOOTER);
        // Run to red or blue line
        runToRedOrBlue(opMode, robot, WAIT_FOR_SEC_LINE);*/
        //beaconTest(opMode, robot, 1.0);
        ultrasonicTest(opMode, robot, 0.5);
    }

    private void runToRedOrBlue(LinearOpMode opMode, BoKHardwareBot robot, double waitForSec) {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToDTMotors(LEFT_MOTOR_POWER, RIGHT_MOTOR_POWER);
            robot.sweeperMotor.setPower(-0.9); // start the sweeper in reverse
            runTime.reset();
/*
            // run till red or blue line or if the user presses stop
            while (opMode.opModeIsActive()) {
                // go to red or blue line
                int current_red = robot.colorSensor.red();
                int current_blue = robot.colorSensor.blue();
                Log.v("BOK", "Red " + current_red + " Blue " + current_blue);
                //int current_green = robot.colorSensor.green();

                while (((current_red < RED_THRESHOLD) && (current_blue < BLUE_THRESHOLD)) && opMode.opModeIsActive() && (runTime.seconds() < waitForSec)) {
                    // Display the color info on the driver station
                    // opMode.telemetry.addData("r: ", current_red + " b: " + current_blue + " g: " + current_green);
                    opMode.telemetry.addData("r: ", current_red + " b: " + current_blue + " sec: " + runTime.seconds());
                    opMode.telemetry.update();
                    Log.v("BOK", "RED " + current_red + " BLUE " + current_blue + " sec: " + runTime.seconds());

                    current_red = robot.colorSensor.red();
                    current_blue = robot.colorSensor.blue();
                    // current_green = robot.colorSensor.green();
                } // while (current_red < RED_THRESHOLD) && (current_blue < BLUE_THRESHOLD)

                robot.setPowerToMotors(0.0f, 0.0f); // stop the robot
                break; // done4
            } // while (opModeIsActive())
            */
        } // if (opModeIsActive())
    }

    private void shootBall(LinearOpMode opMode, BoKHardwareBot robot, double waitForSec) {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToDTMotors(0, 0); // Do not move the robot
            robot.setPowerToShooterMotors(SHOOTER_MOTOR_POWER);   // start the ball shooter
            robot.sweeperMotor.setPower(BoKHardwareBot.SWEEPER_MOTOR_POWER_NORMAL);   // start the sweeper
            runTime.reset();

            // run until the end of the match (driver presses STOP)
            while (opMode.opModeIsActive() && (runTime.seconds() < waitForSec)) {
                opMode.telemetry.addData("Ball shooter: ", "%2.1f sec elapsed", runTime.seconds());
                opMode.telemetry.update();
            } // while (opModeIsActive())

            robot.setPowerToShooterMotors(0); // stop the ball shooter
            robot.sweeperMotor.setPower(0); // stop the sweeper
        } // if (opModeIsActive())
    }


    private void ultrasonicTest(LinearOpMode opMode, BoKHardwareBot robot, double waitForSec) {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
//            robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.setPowerToMotors(LEFT_MOTOR_POWER, RIGHT_MOTOR_POWER);
            //robot.setPowerToSweeper(-1); // start the sweeper in reverse
            runTime.reset();
            robot.setPowerToDTMotors(-0.3,-0.3);

            // run till red or blue line or if the user presses stop
            while (opMode.opModeIsActive()) {
                // go to red or blue line
                /*int current_red = robot.colorSensor.red();
                int current_blue = robot.colorSensor.blue();
                Log.v("BOK", "Red " + current_red + " Blue " + current_blue);
                //int current_green = robot.colorSensor.green();
                */
                distance = robot.rangeSensor.cmUltrasonic();

                while (opMode.opModeIsActive() && (runTime.seconds() < waitForSec)) {
                    opMode.telemetry.addData("BOK", "distance: " + distance);
                    opMode.telemetry.update();
                    distance = robot.rangeSensor.cmUltrasonic();

                } // while (opModeIsActive())


                robot.setPowerToDTMotors(0.0f, 0.0f); // stop the robot
                break; // done
            } // while (opModeIsActive())
        } // if (opModeIsActive())
    }
}
