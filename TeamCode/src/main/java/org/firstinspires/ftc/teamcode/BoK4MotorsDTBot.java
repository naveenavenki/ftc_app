package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Krishna Saxena on 9/24/2016.
 * Extends BoKHardwareBot to implement the 6 Wheel Drive train with 4 DC Motors.
 */
public class BoK4MotorsDTBot extends BoKHardwareBot {

    // CONSTANTS
    // 280 cycles per revolution (CPR); It is a quadrature encoder producing 4 Pulses per Cycle.
    // With 280 CPR, it outputs 1120 PPR. AndyMark 40 Motor Encoder
    // For 360 degrees wheel turn, motor shaft moves 480 degrees (approx)
    protected static final double   COUNTS_PER_MOTOR_REV    = 1120;
    protected static final double   DRIVE_GEAR_REDUCTION    = 1.33;
    protected static final double   WHEEL_DIAMETER_INCHES   = 4.0;
    private static final int DISTANCE_THRESHOLD             = 10;

    // CONSTANTS (strings from the robot config)
    private static final String LEFT_BACK_MOTOR_NAME   = "lb";
    private static final String LEFT_FRONT_MOTOR_NAME  = "lf";
    private static final String RIGHT_BACK_MOTOR_NAME  = "rb";
    private static final String RIGHT_FRONT_MOTOR_NAME = "rf";

    // Drive train motors
    protected DcMotor leftBack;
    protected DcMotor leftFront;
    protected DcMotor rightBack;
    protected DcMotor rightFront;

    // Encoder variables
    private int currentLeftTarget;
    private int currentRightTarget;
    private boolean leftPositive;
    private boolean rightPositive;
    private boolean leftReached;
    private boolean rightReached;

    /*
     * Implement all the abstract methods
     * Initialize the drive system variables.
     * The init() method of the hardware class does all the work here
     */
    protected BoKStatus initDriveTrainMotors(LinearOpMode opMode) {
        currentOpMode = opMode;

        leftBack = opMode.hardwareMap.dcMotor.get(LEFT_BACK_MOTOR_NAME);
        if (leftBack == null) {
            return BoKStatus.BOK_FAILURE;
        }

        leftFront = opMode.hardwareMap.dcMotor.get(LEFT_FRONT_MOTOR_NAME);
        if (leftFront == null) {
            return BoKStatus.BOK_FAILURE;
        }

        rightBack = opMode.hardwareMap.dcMotor.get(RIGHT_BACK_MOTOR_NAME);
        if (rightBack == null) {
            return BoKStatus.BOK_FAILURE;
        }

        rightFront = opMode.hardwareMap.dcMotor.get(RIGHT_FRONT_MOTOR_NAME);
        if (rightFront == null) {
            return BoKStatus.BOK_FAILURE;
        }

        //leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // Drive train is initialized, initialize sensors
        return BoKStatus.BOK_SUCCESS;
    }

    /*
     * Set methods:
     * 1. set power
     * 2. set mode
     * 3. set motor encoder target
     */
    public void setPowerToDTMotors(double left, double right) {
        leftBack.setPower(left);
        rightBack.setPower(right);
        leftFront.setPower(left);
        rightFront.setPower(right);
        currentOpMode.sleep(OPMODE_SLEEP_INTERVAL_MS_SHORT);
    }

    public void setPowerToDTMotors(double leftFrontPower, double leftBackPower,
                                   double rightFrontPower, double rightBackPower) {
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        currentOpMode.sleep(OPMODE_SLEEP_INTERVAL_MS_SHORT);
    }

    public void setModeForDTMotors(DcMotor.RunMode runMode)
    {
        leftBack.setMode(runMode);
        rightBack.setMode(runMode);
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
        currentOpMode.sleep(OPMODE_SLEEP_INTERVAL_MS_SHORT);
    }

    public void setDTMotorEncoderTarget(int leftTarget, int rightTarget)
    {
        // We are NOT using RUN_USING_ENCODER or RUN_TO_POSITION
        setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftReached = rightReached = false;

        currentLeftTarget = leftFront.getCurrentPosition() + leftTarget;
        currentRightTarget = rightFront.getCurrentPosition() + rightTarget;

        currentOpMode.sleep(OPMODE_SLEEP_INTERVAL_MS_SHORT);
        //opMode.idle();

        if (leftTarget > 0)
            leftPositive = true;
        else
            leftPositive = false;

        if (rightTarget > 0)
            rightPositive = true;
        else
            rightPositive = false;

        Log.v("BOK", "START: " + leftFront.getCurrentPosition() +", " + currentLeftTarget + ", " +
                rightFront.getCurrentPosition() + ", " + currentRightTarget);
    }

    /*
     * Get method: Returns true if target is reached
     */
    public boolean getDTCurrentPosition()
    {
        int leftFrontCurrentPos = leftFront.getCurrentPosition();
        int rightFrontCurrentPos = rightFront.getCurrentPosition();

        //Log.v("BOK", "Current " + leftFrontCurrentPos + ", " + leftFront.isBusy() + ", " +
        //             rightFrontCurrentPos + ", " + rightFront.isBusy());

        if (leftPositive) {
            if ((leftFrontCurrentPos >= currentLeftTarget) ||
                    (Math.abs(leftFrontCurrentPos - currentLeftTarget) <= DISTANCE_THRESHOLD)) {
                Log.v("BOK", "RETURNING TRUE!!");
                leftReached = true;
            }
        }
        else {
            if ((leftFrontCurrentPos <= currentLeftTarget) ||
                    (Math.abs(leftFrontCurrentPos - currentLeftTarget) <= DISTANCE_THRESHOLD))
                leftReached = true;
        }

        if (rightPositive) {
            if ((rightFrontCurrentPos >= currentRightTarget) ||
                    (Math.abs(rightFrontCurrentPos - currentRightTarget) <= DISTANCE_THRESHOLD)) {
                Log.v("BOK", "RETURNING TRUE right!!");
                rightReached = true;
            }
        }
        else {
            if ((rightFrontCurrentPos <= currentRightTarget) ||
                    (Math.abs(rightFrontCurrentPos - currentRightTarget) <= DISTANCE_THRESHOLD))
                rightReached = true;
        }

        return rightReached && leftReached;
    }
}
