package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Krishna Saxena on 10/5/2016.
 */
public class League0Auto implements BokAutoTest {

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
    public void initTest(BoKAuto opMode, BoKHardwareBot robot) {
    }

    @Override
    public void runTest(BoKAuto opMode, BoKHardwareBot robot) throws InterruptedException
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

    private void runToRedOrBlue(BoKAuto opMode, BoKHardwareBot robot, double waitForSec) throws InterruptedException {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToMotors(LEFT_MOTOR_POWER, RIGHT_MOTOR_POWER);
            robot.setPowerToSweeper(-1); // start the sweeper in reverse
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

    private void shootBall(BoKAuto opMode, BoKHardwareBot robot, double waitForSec) throws InterruptedException {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToMotors(0, 0); // Do not move the robot
            robot.setPowerToShooter(SHOOTER_MOTOR_POWER);   // start the ball shooter
            robot.setPowerToSweeper(SWEEPER_MOTOR_POWER);   // start the sweeper
            runTime.reset();

            // run until the end of the match (driver presses STOP)
            while (opMode.opModeIsActive() && (runTime.seconds() < waitForSec)) {
                opMode.telemetry.addData("Ball shooter: ", "%2.1f sec elapsed", runTime.seconds());
                opMode.telemetry.update();
            } // while (opModeIsActive())

            robot.setPowerToShooter(0.0f); // stop the ball shooter
            robot.setPowerToSweeper(0.0f); // stop the sweeper
        } // if (opModeIsActive())
    }


    private void ultrasonicTest(BoKAuto opMode, BoKHardwareBot robot, double waitForSec) throws InterruptedException {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
//            robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.setPowerToMotors(LEFT_MOTOR_POWER, RIGHT_MOTOR_POWER);
            //robot.setPowerToSweeper(-1); // start the sweeper in reverse
            runTime.reset();
            robot.setPowerToMotors(-0.3,-0.3);

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


                robot.setPowerToMotors(0.0f, 0.0f); // stop the robot
                break; // done
            } // while (opModeIsActive())
        } // if (opModeIsActive())
    }
}
